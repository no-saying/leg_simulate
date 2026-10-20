"""Train the independent slope-spin residual policy with mjbatch."""

import argparse
import json
import math
import time
from pathlib import Path

import torch
from eval_spin_rl import evaluate_cases, summary
from spin_anchor_env import FRAME_SIZE as ANCHOR_FRAME_SIZE
from spin_anchor_env import OBSERVATION_SCALE as ANCHOR_OBSERVATION_SCALE
from spin_anchor_env import TASK_VERSION as ANCHOR_TASK_VERSION
from spin_anchor_env import AnchorSpinEnv
from spin_asymmetric_env import AsymmetricSpinEnv, PRIVILEGED_NAMES
from spin_asymmetric_env import TASK_VERSION as ASYMMETRIC_TASK_VERSION
from spin_asymmetric_policy import (AsymmetricSpinPolicy, SensorActor, warm_start_sensor_actor,
                                    warm_start_history_residual)
from spin_mixed_env import FRAME_SIZE as MIXED_FRAME_SIZE
from spin_mixed_env import TASK_VERSION as MIXED_TASK_VERSION
from spin_mixed_env import MixedTerrainSpinEnv, validation_cases
from spin_rl_env import (
    ACTION_SCALE,
    FRAME_SIZE,
    HISTORY,
    OBS_SCALE,
    TASK_VERSION,
    SpinRLBatchEnv,
)
from spin_return_env import ReturnTrainingSpinEnv, RETURN_STAGES, return_gate_passes
from spin_standard_env import TASK_VERSION as STANDARD_TASK_VERSION
from spin_standard_env import StandardSpinEnv
from spin_travel_env import FRAME_SIZE as TRAVEL_FRAME_SIZE
from spin_travel_env import OBSERVATION_SCALE, PHYSICAL_ACTION_SCALE, TravelSpinEnv
from spin_travel_env import TASK_VERSION as TRAVEL_TASK_VERSION
from spin_wrench_env import WrenchSpinEnv, ReturnWrenchSpinEnv, OBS_SCALE as WRENCH_OBS_SCALE
from spin_wrench_env import HISTORY_LATENT_SIZE, HISTORY_HIDDEN_SIZE
from spin_wrench_control import ACTION_SCALE as WRENCH_ACTION_SCALE
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CallbackList,
    CheckpointCallback,
)


def warm_start_actor(model, source):
    """Preserve the V5 actor; new per-frame anchor columns initially have zero gain.

    The critic and optimizer start fresh because V6 changes both reward and input.
    """
    old = PPO.load(source, device="cpu")
    old_frame = old.observation_space.shape[0]//HISTORY
    new_frame = model.observation_space.shape[0]//HISTORY
    if old_frame != TRAVEL_FRAME_SIZE or new_frame != ANCHOR_FRAME_SIZE:
        raise ValueError("Actor warm start requires a V4/V5 52-frame model and a V6 56-frame model")
    source_state, target = old.policy.state_dict(), model.policy.state_dict()
    for name, value in source_state.items():
        if name == "mlp_extractor.policy_net.0.weight":
            expanded = torch.zeros_like(target[name])
            for frame in range(HISTORY):
                expanded[:, frame*new_frame:frame*new_frame+old_frame] = value[
                    :, frame*old_frame:(frame+1)*old_frame]
            target[name] = expanded
        elif name.startswith(("mlp_extractor.policy_net.", "action_net.")) or name == "log_std":
            target[name] = value
    model.policy.load_state_dict(target)


class SpinMetricsCallback(BaseCallback):
    def __init__(self, path):
        super().__init__()
        self.path = path

    def _on_step(self):
        for info in self.locals["infos"]:
            if "spin_metrics" in info:
                record = {"timesteps": self.num_timesteps, **info["spin_metrics"],
                          "return": info["episode"]["r"]}
                with self.path.open("a") as output:
                    output.write(json.dumps(record)+"\n")
                for key in ("roll_rms_deg", "roll_peak_deg", "yaw_rmse", "max_drift_m",
                            "yaw_angle_ratio", "failed", "stalled"):
                    self.logger.record_mean("spin/"+key, float(record[key]))
                for key in ("standard_actual_mean_m", "standard_requested_mean_m",
                            "standard_yaw_weight_mean", "standard_drift_weight_mean",
                            "standard_infeasible_fraction", "bounded_spin_pass", "inside_fraction",
                            "final_drift_m", "return_attempts", "return_successes", "unrecovered",
                            "odometry_peak_error_m", "full_rl_control_seconds",
                            "rl_pitch_rms_deg", "rl_pitch_peak_deg"):
                    if key in record:
                        self.logger.record_mean("spin/"+key, float(record[key]))
                for phase, metrics in record["phases"].items():
                    for key in ("roll_rms_deg", "pitch_peak_deg", "yaw_rmse"):
                        self.logger.record_mean(f"phase/{phase}/{key}", metrics[key])
                for name, value in record["reward_totals"].items():
                    self.logger.record_mean("reward/"+name, value)
        return True


class SpinEvaluationCallback(BaseCallback):
    def __init__(self, out, threads, interval=100_000, mixed=False, travel=False, standard=False,
                 anchor=False, asymmetric=False, case_limit=None, episode_seconds=None,
                 control_mode='residual', torque_loop='ideal', linkage_map='native', eval_grid='full',
                 return_env=None, reference_mode='spin', reward_profile='spin'):
        super().__init__()
        self.out, self.threads, self.interval = out, threads, interval
        self.last_eval = 0
        self.mixed = mixed
        self.travel = travel
        self.standard = standard
        self.anchor = anchor
        self.asymmetric = asymmetric
        self.control_mode = control_mode
        self.torque_loop = torque_loop
        self.linkage_map, self.eval_grid = linkage_map, eval_grid
        self.case_limit, self.episode_seconds = case_limit, episode_seconds
        self.return_env = return_env
        self.reference_mode = reference_mode
        self.reward_profile = reward_profile
        self.gate_streak = 0

    def _evaluate(self):
        prefix = self.out.parent / f"{self.out.name}_{self.num_timesteps}"
        self.model.save(prefix)
        terrain = None
        if self.reward_profile == 'pitch':
            cases = [(0, heading, ext, 1) for ext in (-.010, 0, .010)
                     for heading in (0, math.pi/2)]
            terrain = [dict(location='lower_flat', handover='pid', takeover_time=0,
                            fade=.3, dropout=False)]*len(cases)
        elif self.mixed:
            cases, terrain = (validation_cases(slopes=(8,), handovers=('lqr',))
                              if self.eval_grid == 'screen' else validation_cases())
        else:
            cases = [(slope, 0, ext, direction) for slope in (0, 3)
                     for ext in (-0.093, -0.04, 0, 0.04, 0.070) for direction in (-1, 1)]
        if self.case_limit is not None:
            cases = cases[:self.case_limit]
            terrain = terrain[:self.case_limit] if terrain is not None else None
        rows, provenance = evaluate_cases(self.model, cases, terrain, threads=self.threads,
                                          travel=self.travel, standard=self.standard,
                                          anchor=self.anchor,
                                          asymmetric=self.asymmetric,
                                          control_mode=self.control_mode,
                                          torque_loop=self.torque_loop,
                                          linkage_map=self.linkage_map,
                                          reference_mode=self.reference_mode,
                                          reward_profile=self.reward_profile,
                                          episode_seconds=(self.episode_seconds if self.episode_seconds is not None
                                                           else 60 if self.anchor else None),
                                          batch_size=32 if self.travel else 16)
        stats = summary(rows)
        prefix.with_suffix(".eval.json").write_text(json.dumps({
            "task": provenance.get("task", TASK_VERSION),
            "timesteps": self.num_timesteps,
            "evaluation_scope": {"grid": 'pitch_foundation' if self.reward_profile == 'pitch' else self.eval_grid,
                                 "case_limit": self.case_limit,
                                 "episode_seconds_override": self.episode_seconds},
            "provenance": provenance,
            "summary": stats, "results": rows}, indent=2)+"\n")
        for key, value in stats.items():
            if isinstance(value, (int, float)):
                self.logger.record("eval/"+key, value)
        print(f"EVAL steps={self.num_timesteps}: {json.dumps(stats)}", flush=True)
        if self.return_env is not None:
            self._evaluate_return_gate(prefix)
        self.last_eval = self.num_timesteps

    def _evaluate_return_gate(self, prefix):
        stage = self.return_env.return_stage
        cases, terrain = validation_cases(
            slopes=(min(self.return_env.return_max_slope, RETURN_STAGES[stage]['slope']),),
            handovers=('lqr',))
        rows, provenance = evaluate_cases(
            self.model, cases, terrain, threads=self.threads, seed=2000,
            asymmetric=True, anchor=True, travel=True, standard=True,
            episode_seconds=30, disturbances=False, batch_size=32,
            linkage_map=self.linkage_map, return_stage=stage, control_mode=self.control_mode)
        stats = summary(rows)
        passed = return_gate_passes(stats)
        self.gate_streak = self.gate_streak+1 if passed else 0
        # Require two successful post-update validations before changing tasks.
        next_stage = min(stage+1, len(RETURN_STAGES)-1) if self.gate_streak >= 2 else stage
        record = dict(timesteps=self.num_timesteps, stage=stage, passed=passed,
                      consecutive_passes=self.gate_streak, next_stage=next_stage,
                      provenance=provenance, summary=stats, results=rows)
        prefix.with_suffix('.return_gate.json').write_text(json.dumps(record, indent=2)+'\n')
        print(f'RETURN GATE stage={stage} passed={passed}: {json.dumps(stats)}', flush=True)
        if next_stage != stage:
            self.return_env.set_return_stage(next_stage)
            self.gate_streak = 0
        self.out.with_suffix('.curriculum.json').write_text(json.dumps(dict(
            timesteps=self.num_timesteps, stage=next_stage, consecutive_passes=self.gate_streak,
            strict_evaluation=str(prefix.with_suffix('.eval.json'))), indent=2)+'\n')

    def _on_training_start(self):
        self.last_eval = self.num_timesteps

    def _on_step(self):
        # The last rollout still needs its PPO update; score final weights at end.
        if (self.return_env is None and self.num_timesteps-self.last_eval >= self.interval
                and self.num_timesteps < self.model._total_timesteps):
            self._evaluate()
        return True

    def _on_rollout_start(self):
        if (self.return_env is not None and self.num_timesteps-self.last_eval >= self.interval):
            self._evaluate()

    def _on_training_end(self):
        self.model.save(self.out)
        if self.last_eval != self.num_timesteps:
            self._evaluate()


class TravelCurriculumCallback(BaseCallback):
    def __init__(self, env, max_slope):
        super().__init__()
        self.env, self.max_slope, self.stage = env, max_slope, -1

    def apply(self, steps):
        stage = 0 if steps < 65_536 else 1 if steps < 131_072 else 2
        if stage != self.stage:
            lengths, slope = (((0.090, 0.150), 3), ((0.070, 0.180), 5),
                              ((0.047, 0.210), self.max_slope))[stage]
            self.env.leg_ext_range = (0.117-lengths[1], 0.117-lengths[0])
            self.env.max_slope_deg = min(slope, self.max_slope)
            self.stage = stage
            print(f"CURRICULUM stage={stage} steps={steps} lengths={lengths} "
                  f"slope={self.env.max_slope_deg}", flush=True)

    def _on_training_start(self):
        self.apply(self.num_timesteps)

    def _on_step(self):
        self.apply(self.num_timesteps)
        self.logger.record("curriculum/stage", self.stage)
        return True


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--steps", type=int, default=500_000)
    parser.add_argument("--envs", type=int, default=16)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--slope", type=float, default=3, help="Maximum absolute slope in degrees")
    parser.add_argument("--terrain", choices=("mixed", "plane", "travel", "standard", "anchor", "asymmetric"), default="standard")
    parser.add_argument('--control-mode', choices=('residual', 'wrench'), default='residual')
    parser.add_argument('--reference-mode', choices=('spin', 'hold'), default='spin')
    parser.add_argument('--reward-profile', choices=('spin', 'pitch'), default='spin')
    parser.add_argument('--episode-seconds', type=float)
    parser.add_argument('--history-residual', action='store_true',
                        help='Add a zero-initialized history encoder to a migrated residual actor')
    parser.add_argument('--return-curriculum', action='store_true',
                        help='Train return velocity tracking with validation-gated stages')
    parser.add_argument('--return-stage', type=int, choices=(0, 1, 2), default=0)
    parser.add_argument('--reset-critic', action='store_true',
                        help='Reset critic when migrating a history residual actor to a new reward')
    parser.add_argument('--torque-loop', choices=('ideal', 'open', 'pi'), default='ideal')
    parser.add_argument('--linkage-map', choices=('native', 'exact'), default='native')
    parser.add_argument('--eval-grid', choices=('full', 'screen'), default='full',
                        help='Screen: all five locations, three leg lengths, both directions at 8deg/LQR')
    parser.add_argument("--no-curriculum", action="store_true")
    parser.add_argument("--eval-interval", type=int, default=100_000)
    parser.add_argument("--eval-case-limit", type=int, help="Limit evaluation cases for pipeline checks only")
    parser.add_argument("--eval-episode-seconds", type=float, help="Override evaluation duration for pipeline checks")
    parser.add_argument("--checkpoint-interval", type=int, default=32_768)
    parser.add_argument("--rollout", type=int, default=512)
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--warm-start", type=Path, help="Initialize sensor actor from V5/V6; reset critic/optimizer")
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--friction-range", type=float, nargs=2, metavar=("MIN", "MAX"),
                        help="Asymmetric training rollout friction; applied to wheels and terrain")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    wrench = args.control_mode == 'wrench'
    pitch = args.reward_profile == 'pitch'
    if pitch and (not wrench or args.reference_mode != 'hold' or args.slope != 0
                  or args.return_curriculum):
        parser.error('Pitch pretraining requires wrench control, hold reference, slope 0 and no return curriculum')
    if args.return_curriculum and (args.terrain != 'asymmetric' or args.torque_loop != 'ideal'):
        parser.error('Return curriculum requires asymmetric terrain and ideal motors')
    if wrench and args.torque_loop != 'ideal':
        parser.error('Exclusive wrench control does not enable torque PI')
    if args.reset_critic and (not args.warm_start or not args.history_residual):
        parser.error('--reset-critic requires --warm-start and --history-residual')
    if wrench and args.torque_loop == 'open':
        parser.error('Open-loop motor comparison is available for residual control only')
    if args.history_residual and (wrench or args.terrain != 'asymmetric'):
        parser.error('History residual requires asymmetric residual control')
    if args.torque_loop != 'ideal' and args.terrain != 'asymmetric':
        parser.error('Torque loop comparison requires asymmetric terrain')
    if args.linkage_map != 'native' and (args.terrain != 'asymmetric' or wrench):
        parser.error('Exact linkage remapping requires asymmetric residual control')
    if args.eval_grid == 'screen' and args.terrain not in ('mixed', 'travel', 'standard', 'anchor', 'asymmetric'):
        parser.error('Screen grid requires mixed terrain')
    if wrench and (args.terrain != 'asymmetric' or args.warm_start):
        parser.error('Wrench control requires asymmetric terrain and a fresh six-action policy')
    if args.warm_start and (args.resume or args.terrain not in ("anchor", "asymmetric")):
        parser.error("--warm-start requires anchor/asymmetric and cannot be combined with --resume")
    if args.friction_range is not None and (
            args.terrain != 'asymmetric' or not all(math.isfinite(v) for v in args.friction_range)
            or not 0 < args.friction_range[0] <= args.friction_range[1]):
        parser.error('--friction-range requires asymmetric terrain and positive ordered bounds')
    if (args.steps < 1 or args.rollout*args.envs < 2 or args.epochs < 1
            or args.eval_interval < 1 or args.checkpoint_interval < 1
            or (args.eval_case_limit is not None and args.eval_case_limit < 1)
            or (args.eval_episode_seconds is not None and not 0 < args.eval_episode_seconds < float('inf'))):
        parser.error("Positive training sizes required")
    if args.episode_seconds is not None and not 0 < args.episode_seconds < float('inf'):
        parser.error('Positive episode duration required')
    torch.set_num_threads(1)
    if args.out is None:
        names = {"asymmetric": "spin_asymmetric_v7", "anchor": "spin_anchor_v6", "standard": "spin_standard_v5", "travel": "spin_travel_v4",
                 "mixed": "spin_mixed_v3", "plane": "spin_v2"}
        args.out = Path("logs/rl_training") / names[args.terrain]
        if wrench:
            args.out = args.out.with_name('spin_wrench_v8')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    asymmetric = args.terrain == "asymmetric"
    anchor, standard = args.terrain in ("anchor", "asymmetric"), args.terrain in ("standard", "anchor", "asymmetric")
    mixed, travel = args.terrain != "plane", args.terrain in ("travel", "standard", "anchor", "asymmetric")
    env_type = (ReturnWrenchSpinEnv if args.return_curriculum and wrench else ReturnTrainingSpinEnv if args.return_curriculum else WrenchSpinEnv if wrench else AsymmetricSpinEnv if asymmetric else AnchorSpinEnv if anchor else StandardSpinEnv if standard else TravelSpinEnv if travel
                else MixedTerrainSpinEnv if mixed else SpinRLBatchEnv)
    env = env_type(args.envs, args.threads, max_slope_deg=args.slope, reference_mode=args.reference_mode,
                   **({'friction_range': args.friction_range,
                       **({'torque_loop': args.torque_loop, 'reward_profile': args.reward_profile} if wrench else
                          {'residual_torque_loop': args.torque_loop,
                           'linkage_map': args.linkage_map})} if asymmetric else {}),
                   **({'return_stage': args.return_stage} if args.return_curriculum else {}),
                   **({'episode_seconds': args.episode_seconds} if args.episode_seconds is not None else {}))
    env.seed(args.seed)
    evaluation = SpinEvaluationCallback(args.out, args.threads, args.eval_interval, mixed, travel,
                                        standard, anchor, asymmetric,
                                        args.eval_case_limit, args.eval_episode_seconds, args.control_mode,
                                        args.torque_loop, args.linkage_map, args.eval_grid,
                                        env if args.return_curriculum else None, args.reference_mode,
                                        args.reward_profile)
    try:
        if args.resume:
            model = PPO.load(args.resume, env=env, device="cpu")
            if args.history_residual and not getattr(model.policy, 'history_residual', False):
                raise ValueError('Use warm-start to add history compression to an existing actor')
            args.history_residual = getattr(model.policy, 'history_residual', False)
        else:
            model = PPO(AsymmetricSpinPolicy if asymmetric else "MlpPolicy", env, n_steps=args.rollout,
                        batch_size=min(256, args.rollout*args.envs), n_epochs=args.epochs,
                        learning_rate=args.learning_rate, gamma=.99 if pitch else .999,
                        gae_lambda=.95 if pitch else .98,
                        ent_coef=.001 if pitch else .005, policy_kwargs={"net_arch": [256, 256], "log_std_init": -2.3 if wrench else -1.0,
                            **({'actor_size': env.provenance['actor_size'],
                                'history_residual': args.history_residual,
                                **({'history_shape': (env.history.shape[1], env.history.shape[2]),
                                    'history_latent_size': HISTORY_LATENT_SIZE,
                                    'history_hidden_size': HISTORY_HIDDEN_SIZE}
                                   if wrench else {})} if asymmetric else {})},
                        seed=args.seed, device="cpu", verbose=1)
            if wrench:
                # Initialize the network's support outputs, not an external
                # gravity controller. All subsequent motor commands remain RL.
                with torch.no_grad():
                    for channel in (1, 3):
                        model.policy.action_net.bias[channel] = env.robot_weight/(2*WRENCH_ACTION_SCALE[channel])
                env.provenance['actor_initialization'] = 'static support action bias; no runtime teacher or feedforward'
            if args.warm_start:
                if args.history_residual:
                    warm_start_history_residual(model, args.warm_start, reset_critic=args.reset_critic)
                else:
                    (warm_start_sensor_actor if asymmetric else warm_start_actor)(model, args.warm_start)
                print(f"WARM START source={args.warm_start}; "+
                      ('actor preserved; optimizer reset; critic '+('reset' if args.reset_critic else 'preserved')
                       if args.history_residual else 'critic/optimizer reset'), flush=True)
        curriculum = (TravelCurriculumCallback(env, args.slope)
                      if travel and not args.no_curriculum and not args.return_curriculum and not pitch else None)
        if curriculum:
            curriculum.apply(model.num_timesteps)
        model_contract = {
            "task": env.provenance.get('task', TASK_VERSION),
            "frame_size": env.history.shape[-1],
            "history": env.history.shape[1],
            "actor_architecture": ('existing actor + past 7 frames -> FC128 -> FC32 -> latent correction'
                                   if args.history_residual else 'compressed history + current frame' if wrench
                                   else 'MLP'),
            "provenance": env.provenance,
            "observation_shape": list(env.observation_space.shape), "action_shape": list(env.action_space.shape), "policy_hz": 50,
            "physics_hz": 1000,
            "obs_scale": WRENCH_OBS_SCALE.tolist() if wrench else ANCHOR_OBSERVATION_SCALE if anchor else OBSERVATION_SCALE if travel else OBS_SCALE.tolist()+([1, 1] if mixed else []),
            "action_scale": WRENCH_ACTION_SCALE.tolist() if wrench else PHYSICAL_ACTION_SCALE.tolist() if travel else ACTION_SCALE.tolist(),
            "action_semantics": (env.provenance['action_names'] if wrench else ["differential_length_residual", "common_length_reference_offset",
                                  "swing_residual", "common_wheel_torque", "differential_wheel_torque"]
                                 if travel else None),
            "max_slope_deg": args.slope,
            "leg_extension_range_m": list(env.leg_ext_range),
            "gae_lambda": model.gae_lambda, "gamma": model.gamma,
            "stop_seconds": env.recovery_seconds,
            "actor_observation_shape": [env.provenance['actor_size']] if asymmetric else [448] if anchor else list(env.observation_space.shape),
            "critic_privileged_features": list(PRIVILEGED_NAMES) if asymmetric else [],
            "deployment_actor": args.out.name+".actor.pt" if asymmetric else None,
            "yaw_reward": (None if pitch else "anchor_v6_with_fixed_origin_return" if anchor else "standard_v5_mapped_request_weighted_huber" if standard else
                           "travel_v4_extreme_weighted_huber" if travel else "huber_0.6radps"),
            "curriculum": ([{"steps": 0, "lengths_m": [0.090, 0.150], "max_slope_deg": min(3, args.slope)},
                            {"steps": 65536, "lengths_m": [0.070, 0.180], "max_slope_deg": min(5, args.slope)},
                            {"steps": 131072, "lengths_m": [0.047, 0.210], "max_slope_deg": args.slope}]
                           if curriculum else {'stages': list(RETURN_STAGES), 'promotion':
                               'two consecutive fixed return-gate passes; changes apply on reset'}
                           if args.return_curriculum else None),
            "arguments": {key: str(value) if isinstance(value, Path) else value
                          for key, value in vars(args).items()},
        }
        args.out.with_suffix(".json").write_text(json.dumps(model_contract, indent=2)+"\n")
        callback = CallbackList([
            *([curriculum] if curriculum else []),
            SpinMetricsCallback(args.out.with_suffix(".episodes.jsonl")),
            CheckpointCallback(save_freq=max(1, args.checkpoint_interval//args.envs),
                               save_path=str(args.out.parent), name_prefix=args.out.name),
            evaluation,
        ])
        start, initial = time.monotonic(), model.num_timesteps
        model.learn(total_timesteps=args.steps, reset_num_timesteps=False, callback=callback)
        model.save(args.out)
        if asymmetric:
            torch.jit.script(SensorActor(model.policy).eval()).save(str(args.out.with_suffix('.actor.pt')))
        print(f"Saved {args.out}.zip; transitions={model.num_timesteps-initial}; "
              f"elapsed={time.monotonic()-start:.1f}s", flush=True)
    finally:
        env.close()


if __name__ == "__main__":
    main()
