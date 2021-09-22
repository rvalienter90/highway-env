import gym
from stable_baselines3 import A2C
from stable_baselines3.common.vec_env import VecVideoRecorder
import highway_env
import numpy as np
from rl_agents.trainer.evaluation import Evaluation

def evaluate(env,model):
    agent_test = None
    run_directory = None
    options ={ "--episodes_test": 400,
               "--seed":None,
               "--recover":False,
               "--recover-from": False,
               "--no-display": True,
               "--name-from-envconfig": True,
               "--model_save_freq": 50,
               "--video_save_freq" : 1,
               "--create_episode_log": True,
               "--individual_episode_log_level": 2,
               "--create_timestep_log ": False,
               "--individual_reward_tensorboard": False,
               "--create_timestep_log": False,
               "--timestep_log_freq": False,
               "--episodes": 1000,
               "--environment": "stablebaselines_highway_cnn_a2c_policy5"

    }

    evaluation_test = Evaluation(env,
                                 agent_test,
                                 run_directory=run_directory,
                                 num_episodes=int(options['--episodes_test']),
                                 sim_seed=options['--seed'],
                                 recover=options['--recover'] or options['--recover-from'],
                                 display_env=not options['--no-display'],
                                 display_agent=not options['--no-display'],
                                 display_rewards=not options['--no-display'],
                                 training=False,
                                 model =model,
                                 test_stable_baseline=True,
                                 options=options
                                 )
    evaluation_test.test()

if __name__ == '__main__':
    # Train
    env = gym.make('highway-v0')
    env.configure({
        "lanes_count": 3,
        "vehicles_count": 15,
        "observation": {
            "type": "GrayscaleObservation",
            "observation_shape": (128, 64),
            "stack_size": 4,
            "weights": [0.2989, 0.5870, 0.1140],  # weights for RGB conversion
            "scaling": 1.75,
        },
        "policy_frequency": 2,
        "duration": 2*20,
    })
    env.reset()
    train = False
    if train:
        model = A2C('CnnPolicy', env,
                    gamma=0.8,
                    learning_rate=5e-4,
                    verbose=1,
                    tensorboard_log="logs/")
        model.learn(total_timesteps=int(2e5))
        model.save("a2c_highway")
        # model = A2C('CnnPolicy', env).learn(total_timesteps=int(2e5))
        # model.save("a2c_highway_basic")
        # model.save("a2c_highway_policy5")

    # Record video

    # env.configure({"policy_frequency": 15, "duration": 20 * 15})
    # model = A2C.load("a2c_highway_policy5")
    model = A2C.load("a2c_highwayv0")
    # model = A2C.load("a2c_highway_basic")
    # env.configure({"policy_frequency": 15, "duration": 20 * 15})
    # video_length = 2 * env.config["duration"]
    # env = VecVideoRecorder(env, "videos/",
    #                        record_video_trigger=lambda x: x == 0, video_length=video_length,
    #                        name_prefix="dqn-agent")

    evaluate(env, model)

    for _ in range(5):
        obs = env.reset()
        done = False
        while not done:
            action, _ = model.predict(obs)
            print(action)
            print(type(action))
            if action >= 0 and action <5 and isinstance(action, (int, np.integer)):
                obs, reward, done, info = env.step(action)
                env.render()

    # obs = env.reset()
    # for _ in range(video_length + 1):
    #     action, _ = model.predict(obs)
    #     obs, _, _, _ = env.step(action)
    #     env.render()

    env.close()