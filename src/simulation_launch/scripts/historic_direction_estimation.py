# Read positions from output file with FileReader

from utils import FileReader


if __name__ == '__main__':
    output_file = "../output/drop_rate/mds/mds/drop_0.00_seed_42_error_0.0_duration_150_start_30"

    file_reader = FileReader(output_file)

    # Save the positions over time
    positions = file_reader.make_numpy()

    # Compute the directions of the agents


