import dataclasses
import os
import signal
import subprocess
from enum import Enum

from experiment_utils.msg import Manage
import rospy


stop = False


def callback(msg):
    global stop

    if msg.stop:
        stop = True


def signal_handler(sig, frame):
    global stop
    stop = True


def run_experiment(launch_cmd, bag_play, bag_record, topics, manager):
    rosbag_play = subprocess.Popen(
        f"rosbag play --clock {bag_play}",
        shell=True
    )

    # Run the command in a subprocess
    rosbag_record = subprocess.Popen(
            f"rosbag record -O {bag_record} " + " ".join(topics),
            shell=True
        )

    launch_process =  subprocess.Popen(
            launch_cmd,
            shell=True
        )

    rosbag_play.wait()

    # Stop computation cleanly
    manager.publish(Manage(stop=True))

    # Stop recording through SIGINT
    launch_process.wait()
    rosbag_record.kill()


class DirectionsEnum(Enum):
    DISTANCES = "distances"
    DISPLACEMENT = "displacement"
    PARTICLES = "particles"

    def __str__(self):
        return self.value


class AlgorithmEnum(Enum):
    MDS = "mds"
    PF = "particle_filter"

    def __str__(self):
        return self.value


@dataclasses.dataclass
class Config:
    seed: int
    init: bool
    offset: bool
    certainty: bool
    directions: DirectionsEnum


@dataclasses.dataclass
class ParticleConfig(Config):
    n_particles: int
    agents_speed: int
    sensor_std_err: int
    dt: float


@dataclasses.dataclass
class MDSConfig(Config):
    pass


def main():
    input_directory = "/home/quentin/Dev/argos3-ros-triangulation/src/experiment_utils/output"
    output_directory = "/home/quentin/Dev/argos3-ros-triangulation/src/experiment_launch/output"

    seeds = [124]
    directions = [DirectionsEnum.DISPLACEMENT]
    # seeds = [124, 42, 427, 97, 172]

    # TODO: add the possibility to launch multiple batch (seed in computation of the same experiment)
    batch = [1, 2, 3, 4, 5]  # Can use the same seeds as above, won't have any negative impact

    mds = True
    pf = True

    experiment_duration = 120

    mds_inits = [False, True]
    mds_offsets = [False, True]
    mds_certainties = [False, True]

    pf_inits = [False]
    pf_offsets = [False]
    pf_certainties = [False]

    n_particles = 5000
    agents_speed = 30
    sensor_std_err = 10
    dt = 0.1

    experiments = {}

    # For directories in experiments
    experiments_directories = os.listdir(f"{input_directory}")
    for experiment in experiments_directories:
        # List files in the directory
        files = os.listdir(f"{input_directory}/{experiment}")

        # Get the input bag (complete_*.bag)
        experiments[experiment] = [
            f"{input_directory}/{experiment}/",
            [f for f in files if "complete_" in f][0]
        ]

    # Run command to ensure that the time of the ROS bags is used
    os.system("rosparam set /use_sim_time true")

    print("STARTING...")

    rospy.init_node('experiment_manager', anonymous=True)
    publisher = rospy.Publisher('experiment/manage_command', Manage, queue_size=10)

    count = 0
    algorithm_experiments = []

    if mds:
        for init in mds_inits:
            for offset in mds_offsets:  # Offset is not used in simulation
                for certainty in mds_certainties:
                    algorithm_experiments.append((
                        AlgorithmEnum.MDS,
                        MDSConfig(
                            seed=42,
                            init=init,
                            offset=offset,
                            certainty=certainty,
                            directions=DirectionsEnum.DISPLACEMENT
                        )
                    ))

    if pf:
        for init in pf_inits:
            for offset in pf_offsets:  # Offset is not used in simulation
                for certainty in pf_certainties:
                    algorithm_experiments.append((
                        AlgorithmEnum.PF,
                        ParticleConfig(
                            seed=42,
                            init=init,
                            offset=offset,
                            certainty=certainty,
                            directions=DirectionsEnum.DISPLACEMENT,
                            n_particles=n_particles,
                            agents_speed=agents_speed,
                            sensor_std_err=sensor_std_err,
                            dt=dt
                        )
                    ))

    for experiment, inputs in experiments.items():
        for algorithm, config in algorithm_experiments:
            if stop:
                break

            output_dir = f"{output_directory}/{experiment}"
            output_dir += "/mds/mds" if algorithm == AlgorithmEnum.MDS else "/pf/pf"
            output_dir += f"_init" if config.init else ""
            output_dir += f"_offset" if config.offset else ""
            output_dir += f"_certainty" if config.certainty else ""

            input_dir = inputs[0]
            input_file = inputs[1]

            # Create output directory if it doesn't exist
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)

            # Possibility to have different batch (various seeds) for the same experiment configuration
            sub_experiments = []
            for seed in seeds:
                for direction in directions:
                    sub_experiments.append(
                        (seed, direction)
                    )

            number_of_experiments = len(experiments) * len(algorithm_experiments) * len(sub_experiments)

            for seed, direction in sub_experiments:
                count += 1
                output_file = f"seed_{seed}_direction_{direction}.bag"

                # If input file already exists, skip
                if output_file in os.listdir(output_dir):
                    print(f"SKIP: {count}/{number_of_experiments}, seed = {seed}...")
                    continue
                else:
                    print(f"RUN: {algorithm}, {count}/{number_of_experiments}, seed = {seed}...")

                arguments = ""

                arguments += f" b:=fbB"
                arguments += f" n:=4"

                arguments += f" experiment_duration:=\"{experiment_duration + 1}\""

                arguments += f" random_seed:=\"{seed}\""

                arguments += f" input_dir:=\"{input_dir}\""
                arguments += f" input_file:=\"{input_file}\""

                arguments += f" output_dir:=\"{output_dir}\""
                arguments += f" output_file:=\"{output_file}\""

                arguments += f" init:=\"{config.init}\""
                arguments += f" offset:=\"{config.offset}\""
                arguments += f" certainty:=\"{config.certainty}\""
                arguments += f" directions:=\"{config.directions}\""

                if isinstance(config, ParticleConfig) and algorithm == AlgorithmEnum.PF:
                    arguments += f" n_particles:=\"{config.n_particles}\""
                    arguments += f" agents_speed:=\"{config.agents_speed}\""
                    arguments += f" sensor_std_err:=\"{config.sensor_std_err}\""
                    arguments += f" dt:=\"{dt}\""

                launch_command = f"roslaunch experiment_launch {algorithm}_experiment.launch" + arguments
                run_experiment(
                    launch_cmd=launch_command,
                    bag_play=f"{input_dir}/{input_file}",
                    bag_record=f"{output_dir}/{output_file}",
                    topics=[
                        "/compute/positions",
                        "/experiment/positions"
                    ],
                    manager=publisher
                )

    print("FINISHED")


if __name__ == "__main__":
    # Set signal handler
    signal.signal(signal.SIGINT, signal_handler)

    main()
