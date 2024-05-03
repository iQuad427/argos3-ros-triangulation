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


@dataclasses.dataclass
class ParticleConfig(Config):
    n_particles: int
    agents_speed: int
    sensor_std_err: int
    dt: float


@dataclasses.dataclass
class MDSConfig(Config):
    init: bool
    offset: bool
    certainty: bool


def main():
    input_directory = "/home/quentin/Dev/argos3-ros-triangulation/src/simulation_experiments/output/drop_rate"
    output_directory = "/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/output/drop_rate"

    # seeds = [124]
    seeds = [124, 42, 427, 97, 172]
    # drops = [0.00]
    drops = [0.00, 0.25, 0.50, 0.75, 0.90, 0.95, 0.96, 0.97, 0.98, 0.99]
    # errors = [0.00]
    errors = [0.00, 0.05, 0.10, 0.15]

    # TODO: add the possibility to launch multiple batch (seed in computation of the same experiment)
    batch = [1, 2, 3, 4, 5]  # Can use the same seeds as above, won't have any negative impact

    mds = True
    pf = False

    experiment_duration = 60

    n_particles = 5000
    agents_speed = 0
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
        for init in [False]:
            for offset in [False]:  # Offset is not used in simulation
                for certainty in [False]:
                    mds_experiments.append(MDSConfig(
                        seed=42,
                        init=init,
                        offset=offset,
                        certainty=certainty
                    ))

    for config in mds_experiments:
        output_dir = f"{output_directory}/mds/dynamic_mds"
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
            file_name = f"drop_{drop:0.2f}_seed_{seed}_error_{error}_duration_{150}_start_{30}"

            # If input file already exists, skip
            if file_name in os.listdir(output_dir):
                print(f"SKIP: {count}/{len(mds_experiments) * len(experiments)}, drop_rate = {drop}, seed = {seed}, error = {error}...")
                continue

            arguments = ""

            arguments += f" experiment_duration:=\"{experiment_duration + 1}\""

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

    if pf:
        for init in [True, False]:
            for offset in [True, False]:
                for certainty in [True, False]:
                    count += 1

                    output_dir = f"{output_directory}/static_convergence_pf"
                    output_dir += f"_init" if init else ""
                    output_dir += f"_offset" if offset else ""
                    output_dir += f"_certainty" if certainty else ""

                    try:
                        os.mkdir(output_dir)
                    except FileExistsError:
                        # If one file in the directory contains the "seed_{seed}" string, skip the experiment
                        if any((f"seed_{seed}" in file and f"particle_{n_particles}" in file) for file in os.listdir(output_dir)):
                            print(f"static_convergence_pf_init_{init}_offset_{offset}_certainty_{certainty} skipped.")
                            continue

                    for batch in range(1, 6):
                        print(
                            f"{count}/8 : init = {init}, offset = {offset}, certainty = {certainty}, batch = {batch}"
                        )

                        arguments = ""

                        # arguments += f" experiment_duration:=\"5\""

                        arguments += f" random_seed:=\"{seed}\""

                        arguments += f" input_dir:=\"{input_directory}\""
                        arguments += f" input_file:=\"raw_data_robot_4_try_3_delay_0_batch_{batch}\""

                        arguments += f" output_dir:=\"{output_dir}\""
                        arguments += f" output_file:=\"seed_{seed}_robot_4_try_3_delay_0_particle_{n_particles}_dt_{dt}_err_{sensor_std_err}_batch_{batch}\""

                        arguments += f" init:=\"{init}\""
                        arguments += f" offset:=\"{offset}\""
                        arguments += f" certainty:=\"{certainty}\""

                        arguments += f" n_particles:=\"{n_particles}\""
                        arguments += f" agents_speed:=\"{agents_speed}\""
                        arguments += f" sensor_std_err:=\"{sensor_std_err}\""
                        arguments += f" dt:=\"{dt}\""

                        run_command("roslaunch simulation_compute particle_filter_statistics.launch" + arguments)

    print("FINISHED")


if __name__ == "__main__":
    main()
