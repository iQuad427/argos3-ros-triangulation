import dataclasses
import datetime
from typing import List

import numpy as np
import matplotlib.pyplot as plt

from src.simulation_compute.scripts.utils import rotate_and_translate

from tqdm import tqdm


@dataclasses.dataclass
class Position:
    id: int
    x: float
    y: float

    def __repr__(self):
        return f"{self.id},{self.x},{self.y}"


@dataclasses.dataclass
class Memory:
    positions: List[Position]
    timestamp: float

    def __repr__(self):
        buffer = f"{self.timestamp}&"
        for position in self.positions:
            buffer += f"{position}#"

        return buffer[:-1]


class FileReader:
    """
    Read a file like the following :
    type=relative_time=time&id,x,y#id,x,y#id,x,y#id,x,y ...

    And allow for doing some operations on it.
    """
    def __init__(self, file_path):
        self.file_path = file_path
        self.file_name = file_path.split("/")[-1]
        self.data, self.time = self._read_file(file_path)

    def _read_file(self, file_path):
        with open(file_path, "r") as f:
            lines = f.readlines()

        data = []
        time = []
        # By stem of 2 because we have 2 lines for each time
        for j in range(0, len(lines), 2):
            estimation_line = lines[j].split("\n")[0].split("=")
            simulation_line = lines[j + 1].split("\n")[0].split("=")

            time.append(float(estimation_line[1]))
            time_step = []

            for line in [estimation_line, simulation_line]:
                positions = line[2].split("&")[1].split("#")
                positions = [position.split(",") for position in positions]
                positions = [
                    Position(int(position[0]), float(position[1]), float(position[2])) for position in positions
                ]
                time_step.append(
                    Memory(
                        positions,
                        float(line[2].split("&")[0])
                    )
                )

            data.append(time_step)

        return data, time

    def make_numpy(self):
        data = []

        for step in self.data:
            estimation = np.array([[position.x, position.y] for position in step[0].positions])
            simulation = np.array([[position.x, position.y] for position in step[1].positions])

            estimation = rotate_and_translate(np.array(simulation), np.array(estimation))

            data.append([estimation, simulation])

        return data


if __name__ == '__main__':
    # Seed 1 : 124
    # Seed 2 : 042
    # Seed 3 : 427
    # Seed 4 : 097
    # Seed 5 : 172

    limit = 300
    file_directory = f"../output/experiments/static"

    errors = [0.1]
    drops = [0.00, 0.50, 0.90, 0.95, 0.96, 0.97, 0.98, 0.985]

    for error in errors:
        for drop in tqdm(drops):
            # Plot the Mean Square Error of the positions
            mean_square_error = []
            time = np.arange(0, limit) / 5

            # Read the file
            for batch in range(1, 6):
                file_name = f"drop_{drop:0.2f}_err_{error}_batch_{batch}" if len(str(drop)) <= len(f"{drop:0.2f}") else f"drop_{drop:0.3f}_err_0.1_speed_30_batch_{batch}"

                try:
                    file_reader = FileReader(f"{file_directory}/{file_name}")
                except:
                    continue

                mses = []

                # print("[0] File :", file_reader.file_path)
                # print("[0] File name :", file_reader.file_name)

                for est, sim in file_reader.make_numpy():
                    mse = np.mean(np.square(est - sim))

                    mses.append(mse)

                # If MSEs is shorter than limit, add last value to fill up
                while len(mses) < limit:
                    mses = mses + [mses[-1]]

                mses = mses[:limit]

                mean_square_error.append(mses)

            if not mean_square_error:
                continue

            # Average the result for each time_step
            mean_square_error = np.mean(np.array(mean_square_error), axis=0)

            # Plot the Mean Square Error
            label = f"Drop rate: {drop:0.2f}" if len(str(drop)) <= len(f"{drop:0.2f}") else f"Drop rate: {drop:0.3f}"
            label_err = f", err: {error}" if len(errors) > 1 else ""
            plt.plot(time, mean_square_error[:limit], label=label+label_err)

    # Axis labels
    plt.xlabel("Time (s)")
    plt.ylabel("Mean Square Error (cm²)")

    plt.title(f"MSE of Positions: {file_directory.split('/')[-2]}, {file_directory.split('/')[-1]}")
    plt.legend()
    plt.savefig(f"../output/mse_drop_{file_directory.split('/')[-1]}.png", dpi=300)
    plt.show()
