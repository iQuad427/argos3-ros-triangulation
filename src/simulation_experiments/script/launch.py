import os
import subprocess
import time
import rospy

from simulation_utils.msg import Manage


def run_command(prior, commands, manager):
    main_process = subprocess.Popen(prior, shell=True)

    time.sleep(3)

    # Run the command in a subprocess
    processes = []
    for command in commands:
        process = subprocess.Popen(command, shell=True)

        processes.append(process)

    print("Waiting for processes to finish...")

    # Wait for subprocess to finish and kill prior
    [process.wait() for process in processes]

    print("Processes finished!")

    # Send sigint to main_process
    manager.publish(Manage(stop=True))

    print("Waiting for main process to finish...")

    main_process.wait()


def main():
    # Seed 1 : 124
    # Seed 2 : 42
    # Seed 3 : 427
    # Seed 4 : 97
    # Seed 5 : 172

    input_directory = "/home/quentin/Dev/argos3-ros-triangulation/src/simulation_experiments/output/drop_rate"
    # seeds = ["124", "42", "427", "97", "172"]
    seeds = ["124"]
    # drops = [0.90, 0.95, 0.96, 0.97, 0.98, 0.99]
    drops = [0.90]
    # errors = [0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30]
    errors = [0.00]

    experiment_duration = 120
    experiment_start = 0
    iteration_rate = 20

    total_experiments = len(seeds) * len(drops) * len(errors)

    experiment = "save_experiment_data"
    simulation = False

    rospy.init_node('simulation_manager', anonymous=True)
    publisher = rospy.Publisher('simulation/manage_command', Manage, queue_size=10)

    # Create all combinations of seeds and drop rates
    experiments = []
    for drop_rate in drops:
        for seed in seeds:
            for error in errors:
                experiments.append((drop_rate, seed, error))

    count = 0
    for drop_rate, seed, error in experiments:
        count += 1
        input_file = f"drop_{drop_rate:0.2f}_seed_{seed}_error_{error}_duration_{experiment_duration}_start_{experiment_start}"

        # If input file already exists, skip
        if input_file in os.listdir(input_directory):
            print(f"SKIP: {count}/{total_experiments}, drop_rate = {drop_rate}, seed = {seed}...")
            continue

        # Create a new temporary file which correspond to .argos template and replace the seed value
        with open("/home/quentin/Dev/argos3-ros-triangulation/src/simulation_experiments/output/template.argos", "r") as file:
            data = file.readlines()

        # Cycle through the line to replace the placeholders
        for i, line in enumerate(data):
            if "&seed&" in line:
                data[i] = line.replace("&seed&", str(seed))
            if "&iteration&" in line:
                data[i] = line.replace("&iteration&", str(iteration_rate))
            if "&drop_rate&" in line:
                data[i] = line.replace("&drop_rate&", str(drop_rate))
            if "&duration&" in line:
                data[i] = line.replace("&duration&", str(experiment_duration))
            if "&start_time&" in line:
                data[i] = line.replace("&start_time&", str(experiment_start * 10))
            if "&sensor_error&" in line:
                data[i] = line.replace("&sensor_error&", str(error))

        with open("/home/quentin/Dev/argos3-ros-triangulation/src/simulation_experiments/output/tmp.argos", "w+") as file:
            file.writelines(data)

        print(f"STARTING: {count}/{total_experiments}, drop_rate = {drop_rate}, seed = {seed}...")

        run_command(
            f"roslaunch simulation_experiments {experiment}.launch"
            + f" simulation:={simulation}"
            + f" experiment_duration:={experiment_duration}"
            + f" experiment_start:={experiment_start}"
            + f" iteration_rate:={iteration_rate}"
            + f" input_dir:={input_directory}"
            + f" input_file:={input_file}"
            + f" random_seed:={seed}",
            [
                f"argos3 -c /home/quentin/Dev/argos3-ros-triangulation/src/simulation_experiments/output/tmp.argos",
            ],
            manager=publisher
        )


if __name__ == "__main__":
    main()
