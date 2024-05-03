import os
import subprocess
import time


def run_command(prior, commands):
    main_process = subprocess.Popen(prior, shell=True)

    time.sleep(5)

    # Run the command in a subprocess
    processes = []
    for command in commands:
        process = subprocess.Popen(command, shell=True)

        processes.append(process)

    # Wait for subprocess to finish and kill prior
    [process.wait() for process in processes]
    # Send sigint to main_process
    main_process.send_signal(signal.SIGINT)

    main_process.wait()


def main():
    input_directory = "/home/quentin/Dev/argos3-ros-triangulation/src/saving_experiments/output/fast_data"
    seeds = [42]
    drops = [0.50]
    # drops = [0.50, 0.90, 0.95, 0.97, 0.98, 0.99]

    experiment_duration = 150
    experiment_start = 30

    total_experiments = len(seeds) * len(drops)

    experiment = "save_fast_experiment_data"
    simulation = False

    count = 0
    for drop_rate in drops:
        for seed in seeds:
            count += 1
            input_file = f"drop_{drop_rate:0.2f}_seed_{seed}"

            # Create a new temporary file which correspond to .argos template and replace the seed value
            with open("/home/quentin/Dev/argos3-ros-triangulation/src/saving_experiments/output/template.argos", "r") as file:
                data = file.readlines()

            # Cycle through the line to replace the following space holder:
            # &seed& -> seed
            # &drop_rate& -> drop_rate
            for i, line in enumerate(data):
                if "&seed&" in line:
                    data[i] = line.replace("&seed&", str(seed))
                if "&drop_rate&" in line:
                    data[i] = line.replace("&drop_rate&", str(drop_rate))
                if "&duration&" in line:
                    data[i] = line.replace("&duration&", str(experiment_duration))
                if "&start_time&" in line:
                    data[i] = line.replace("&start_time&", str(experiment_start * 10))

            with open("/home/quentin/Dev/argos3-ros-triangulation/src/saving_experiments/output/tmp.argos", "w+") as file:
                file.writelines(data)

            print(f"STARTING: {count}/{total_experiments}, drop_rate = {drop_rate}, seed = {seed}...")
            print(f"TIME LEFT: {(total_experiments - (count - 1)) * experiment_duration / 60} minutes")

            run_command(
                f"roslaunch saving_experiments {experiment}.launch"
                + f" simulation:={simulation}"
                + f" experiment_duration:={experiment_duration}"
                + f" experiment_start:={experiment_start}"
                + f" input_dir:={input_directory}"
                + f" input_file:={input_file}"
                + f" random_seed:={seed}",
                [
                    f"argos3 -c /home/quentin/Dev/argos3-ros-triangulation/src/saving_experiments/output/tmp.argos",
                ]
            )


if __name__ == "__main__":
    main()
