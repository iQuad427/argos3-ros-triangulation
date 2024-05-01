import os
import subprocess
import time


def run_command(commands, duration=10):

    # Run the command in a subprocess
    processes = []
    for command in commands:
        process = subprocess.Popen(command, shell=True)

        processes.append(process)

    # Wait for subprocess to finish
    return [process.wait() for process in processes]


def main():
    input_directory = "/home/quentin/Dev/argos3-ros-triangulation/src/saving_experiments/output/raw_data"
    seeds = [42, 124]

    experiment_duration = 10
    experiment_start = 5

    for drop_rate in [0.50, 0.90]:
        for batch in range(1, 3):
            seed = seeds[batch - 1]
            input_file = f"drop_{drop_rate:0.2f}_seed_{seed}_batch_{batch}"

            # Create a new temporary file which correspond to .argos template and replace the seed value
            with open("/home/quentin/Dev/argos3-ros-triangulation/src/saving_experiments/output/template.argos", "r") as file:
                data = file.readlines()

            # Cycle through the line to replace the following space holder:
            # &seed& -> seed
            # &drop_rate& -> drop_rate
            # &time& -> experiment_duration
            for i, line in enumerate(data):
                if "&seed&" in line:
                    data[i] = line.replace("&seed&", str(seed))
                if "&drop_rate&" in line:
                    data[i] = line.replace("&drop_rate&", str(drop_rate))

            with open("/home/quentin/Dev/argos3-ros-triangulation/src/saving_experiments/output/tmp.argos", "w+") as file:
                file.writelines(data)

            print(f"STARTING: drop_rate = {drop_rate}, batch = {batch}, seed = {seed}...")

            run_command([
                f"argos3 -c /home/quentin/Dev/argos3-ros-triangulation/src/saving_experiments/output/tmp.argos",
                f"roslaunch saving_experiments save_experiment_data.launch"
                + f" experiment_duration:={experiment_duration}"
                + f" experiment_start:={experiment_start}"
                + f" input_dir:={input_directory}"
                + f" input_file:={input_file}"
                + f" random_seed:={seed}"
            ])


if __name__ == "__main__":
    main()
