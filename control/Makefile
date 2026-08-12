MUJOCO_DIR ?= $(HOME)/mujoco
CFLAGS   = -O2 -Wall -Wextra
CFLAGS  += $(EXTRA_CFLAGS)
INCLUDES = -I$(MUJOCO_DIR)/include -I. -Iinclude -Iapp -Ibalance -Ilowlevel \
           -Isim -Iruntime -Idashcam
LDFLAGS  = -L$(MUJOCO_DIR)/build/lib -lmujoco -lglfw -lGL -lm -lpthread -ldl
RPATH    = -Wl,-rpath,$(MUJOCO_DIR)/build/lib

SRCS = app/controller.c \
       app/app_sim.c app/app_keys.c app/app_headless.c app/app_gui.c \
       balance/balance.c balance/balance_api.c balance/balance_util.c \
       balance/balance_core.c \
       controllers/lqr/lqr_controller.c \
       controllers/energy/energy_controller.c \
       controllers/supervisor/controller_supervisor.c \
       controllers/rl_jump/rl_jump_net.c \
       controllers/rl_jump/rl_jump_controller.c \
       lowlevel/pid.c lowlevel/wheel_t.c lowlevel/leg_t.c \
       sim/sensors.c sim/telemetry.c sim/trail.c runtime/robot_control.c \
       dashcam/dashcam.c
BUILD_DIR = build
OBJS = $(addprefix $(BUILD_DIR)/,$(SRCS:.c=.o))

all: ../controller

../controller: $(OBJS)
	gcc $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS) $(RPATH)

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	gcc $(CFLAGS) $(INCLUDES) -c $< -o $@

run: ../controller
	cd .. && ./controller

PORTABLE_SRCS = balance/balance.c \
                lowlevel/pid.c lowlevel/wheel_t.c lowlevel/leg_t.c \
                balance/balance_api.c balance/balance_util.c balance/balance_core.c \
                controllers/lqr/lqr_controller.c \
                controllers/energy/energy_controller.c \
                controllers/supervisor/controller_supervisor.c \
                controllers/rl_jump/rl_jump_net.c \
                controllers/rl_jump/rl_jump_controller.c \
                runtime/robot_control.c dashcam/dashcam.c
PORTABLE_INCLUDES = -Iinclude -I. -Ibalance -Ilowlevel -Iruntime -Idashcam -Icontrollers

## 移植性检查: 控制核心(平衡/低层/日志)不依赖 MuJoCo/GLFW 头文件。
check-portable:
	@echo "== Compiling control core WITHOUT MuJoCo/GLFW headers =="
	@mkdir -p build/portable
	@for src in $(PORTABLE_SRCS); do \
		obj=build/portable/$$(echo $$src | tr '/' '_' | sed 's/\.c$$/.o/'); \
		gcc $(CFLAGS) $(PORTABLE_INCLUDES) -c $$src -o $$obj || exit 1; \
		echo "  ok: $$src"; \
	done
	@echo "== OK: control core has no MuJoCo/GLFW dependency =="

clean:
	rm -rf $(BUILD_DIR) ../controller

.PHONY: all run check-portable clean
