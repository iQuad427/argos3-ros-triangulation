import dataclasses
import os
import subprocess


def run_command(command):
    # Run the command in a subprocess
    process = subprocess.Popen(command, shell=True)

    # Wait for subprocess to finish
    process.wait()


@dataclasses.dataclass
class Config:
    seed: int
    init: bool
    offset: bool
    certainty: bool


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
    input_directory = "/home/quentin/Dev/argos3-ros-triangulation/src/simulation_experiments/output/directions"
    output_directory = "/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/output/final_slow"

    # seeds = [124]
    seeds = [124, 42, 427, 97, 172]
    drops = [0.50]
    # drops = [0.00, 0.25, 0.50, 0.75, 0.90, 0.95, 0.96, 0.97, 0.98, 0.99]
    # errors = [0.15]
    errors = [0.00, 0.05, 0.10, 0.15]

    # TODO: add the possibility to launch multiple batch (seed in computation of the same experiment)
    batch = [1, 2, 3, 4, 5]  # Can use the same seeds as above, won't have any negative impact

    mds = True
    pf = True

    experiment_duration = 65

    iterations = 100
    duration = 120
    start = 0

    inits = [False, True]
    offsets = [False]  # Offset is not used in simulation
    certainties = [False, True]

    n_particles = 5000
    agents_speed = 30
    sensor_std_err = 10
    dt = 0.1

    print("STARTING...")

    count = 0
    try:
        os.mkdir(output_directory)
    except FileExistsError:
        pass

    print("PHASE 1: MULTIDIMENSIONAL SCALING")

    mds_experiments = []
    if mds:
        for init in inits:
            for offset in offsets:  # Offset is not used in simulation
                for certainty in certainties:
                    mds_experiments.append(MDSConfig(
                        seed=42,
                        init=init,
                        offset=offset,
                        certainty=certainty
                    ))

    for config in mds_experiments:
        output_dir = f"{output_directory}/mds/mds"
        output_dir += f"_init" if config.init else ""
        output_dir += f"_certainty" if config.certainty else ""

        try:
            os.mkdir(output_dir)
        except FileExistsError:
            pass

        experiments = []
        for drop in drops:
            for seed in seeds:
                for error in errors:
                    experiments.append((drop, seed, error))

        for drop, seed, error in experiments:
            count += 1
            file_name = f"drop_{drop:0.2f}_iteration_{iterations}_seed_{seed}_error_{error}_duration_{duration}_start_{start}"

            # If input file already exists, skip
            if file_name in os.listdir(output_dir):
                print(f"SKIP: {count}/{len(mds_experiments) * len(experiments)}, drop_rate = {drop}, seed = {seed}, error = {error}...")
                continue
            else:
                print(f"RUN: {count}/{len(mds_experiments) * len(experiments)}, drop_rate = {drop}, seed = {seed}, error = {error}...")

            arguments = ""

            arguments += f" experiment_duration:=\"{experiment_duration + 1}\""
            arguments += f" iteration_rate:=\"{iterations}\""

            arguments += f" random_seed:=\"{seed}\""

            arguments += f" input_dir:=\"{input_directory}\""
            arguments += f" input_file:=\"{file_name}\""

            arguments += f" output_dir:=\"{output_dir}\""
            arguments += f" output_file:=\"{file_name}\""

            arguments += f" init:=\"{config.init}\""
            arguments += f" offset:=\"{config.offset}\""
            arguments += f" certainty:=\"{config.certainty}\""

            run_command("roslaunch simulation_launch mds_statistics.launch" + arguments)

    print("PHASE 2: PARTICLE FILTER")

    pf_experiments = []
    if pf:
        for init in inits:
            for offset in offsets:  # Offset is not used in simulation
                for certainty in certainties:
                    pf_experiments.append(ParticleConfig(
                        seed=42,
                        init=init,
                        offset=offset,
                        certainty=certainty,
                        n_particles=n_particles,
                        agents_speed=agents_speed,
                        sensor_std_err=sensor_std_err,
                        dt=dt
                    ))

    for config in pf_experiments:
        output_dir = f"{output_directory}/pf_particles_{n_particles}_std_{sensor_std_err}_dt_{dt}/pf"
        output_dir += f"_init" if config.init else ""
        output_dir += f"_certainty" if config.certainty else ""

        try:
            os.mkdir(output_dir)
        except FileExistsError:
            pass

        experiments = []
        for drop in drops:
            for seed in seeds:
                for error in errors:
                    experiments.append((drop, seed, error))

        for drop, seed, error in experiments:
            count += 1
            file_name = f"drop_{drop:0.2f}_iteration_{iterations}_seed_{seed}_error_{error}_duration_{duration}_start_{start}"

            # If input file already exists, skip
            if file_name in os.listdir(output_dir):
                print(f"SKIP: {count}/{len(pf_experiments) * len(experiments)}, drop_rate = {drop}, seed = {seed}, error = {error}...")
                continue
            else:
                print(f"RUN: {count}/{len(pf_experiments) * len(experiments)}, drop_rate = {drop}, seed = {seed}, error = {error}...")

            arguments = ""

            arguments += f" experiment_duration:=\"{experiment_duration + 1}\""
            arguments += f" iteration_rate:=\"{iterations}\""

            arguments += f" random_seed:=\"{seed}\""

            arguments += f" input_dir:=\"{input_directory}\""
            arguments += f" input_file:=\"{file_name}\""

            arguments += f" output_dir:=\"{output_dir}\""
            arguments += f" output_file:=\"{file_name}\""

            arguments += f" init:=\"{config.init}\""
            arguments += f" offset:=\"{config.offset}\""
            arguments += f" certainty:=\"{config.certainty}\""

            arguments += f" n_particles:=\"{n_particles}\""
            arguments += f" agents_speed:=\"{agents_speed}\""
            arguments += f" sensor_std_err:=\"{sensor_std_err}\""
            arguments += f" dt:=\"{dt}\""

            run_command("roslaunch simulation_launch particle_filter_statistics.launch" + arguments)

    print("FINISHED")


if __name__ == "__main__":
    main()
