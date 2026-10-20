"""Reward densities in physical units, with no simulation state access."""

import numpy as np


def pitch_balance_reward(pitch, pitch_rate, roll, length_error, actions, action_delta):
    """Dense, bounded rewards for learning balance before return or spin tasks."""
    terms = {
        'pitch_upright': 6/(1+(pitch/np.deg2rad(10))**2),
        'pitch_rate': -.5*np.tanh((pitch_rate/1.5)**2),
        'roll_guard': -.15*np.tanh((roll/np.deg2rad(10))**2),
        'height_guard': -.3*np.tanh((length_error/.025)**2),
        'effort': -.02*np.mean(actions**2, axis=1),
        'smooth': -.03*np.mean(action_delta**2, axis=1),
    }
    # Positive living density and a fixed terminal cost favor longer survival.
    return 1+sum(terms.values()), terms
