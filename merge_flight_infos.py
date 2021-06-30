import csv
import numpy as np
import os


def merge_flight_infos(original_datasets, fi_folder, run, it):
    # Flight info files
    files = os.listdir(fi_folder)
    new_files = np.setdiff1d(files, original_datasets)
    original_file = new_files[0]
    if run == 0:
        with open(os.path.join(fi_folder, original_file)) as f_start:
            csv_reader = csv.reader(f_start, delimiter=',')
            counter = -1
            rows = []
            for row in csv_reader:
                counter += 1
                rows.append(row)
        if counter != 1:
            with open(os.path.join(fi_folder, original_file), 'w', encoding='UTF8', newline='') as f_start:
                csv_writer = csv.writer(f_start, delimiter=',')
                csv_writer.writerow(rows[0])
                it = 1
                for i in range(1, len(rows)):
                    row = rows[i]
                    row[0] = str(it)
                    csv_writer.writerow(row)
                    it += 1
    else:
        it = it

    drone_created_files = new_files[1:]
    with open(os.path.join(fi_folder, original_file), 'a', encoding='UTF8', newline='') as f_original:
        original_writer = csv.writer(f_original, delimiter=',')
        for file in drone_created_files:
            with open(os.path.join(fi_folder, file)) as f_run:
                csv_reader = csv.reader(f_run, delimiter=',')
                header = True
                for row in csv_reader:
                    if header:
                        header = False
                    else:
                        row[0] = str(it)
                        original_writer.writerow(row)
                        it += 1
            os.remove(os.path.join(fi_folder, file))
    return it
