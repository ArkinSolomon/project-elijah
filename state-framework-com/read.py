import csv
from typing import Any

from framework.readable.readable_file import ReadableFile
from framework.state_framework import StateFramework

file = '/Users/arkinsolomon/Desktop/launch-80c9d54730aac987'
# csv_path = '/Users/arkinsolomon/Downloads/launch-data-from-fullscale.csv'

with open(file, 'rb') as f:
    #, open(csv_path, 'w', newline='') as csv_file:
    readable = ReadableFile(f)
    f.read(9)
    sf = StateFramework.generate_framework_configuration(readable)

    # writer = csv.writer(csv_file)

    # var_defs = [(var_def.display_name, var_def.display_unit, var_def.variable_id) for var_def in sf.variable_definitions]
    # writer.writerow([f'{var_def[0]} {var_def[1]}' for var_def in var_defs])

    while sf.update(readable, 1) > 0:
        print(sf.state)
        # data: list[Any] = []
        #
        # for var_def in var_defs:
        #     data.append(sf.state[var_def[2]])
        #
        # writer.writerow(data)