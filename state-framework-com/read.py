import csv
from typing import Any

from framework.readable.readable_file import ReadableFile
from framework.state_framework import StateFramework

file = '/Users/arkinsolomon/Downloads/Payload - Primary/pdf'
csv_path = '/Users/arkinsolomon/Downloads/dump.csv'

with open(file, 'rb') as f, open(csv_path, 'w', newline='') as csv_file:
    readable = ReadableFile(f)
    f.read(9)
    sf = StateFramework.generate_framework_configuration(readable)

    print(f'Reading data for {sf.application_name}')

    writer = csv.writer(csv_file)

    var_defs = [(var_def.display_name, var_def.display_unit, var_def.variable_id) for var_def in sf.variable_definitions]
    writer.writerow([f'{var_def[0]} {var_def[1]}' for var_def in var_defs])

    last_seq: int = -1
    while True:
        packets_read, phase_changed, logs = sf.update(readable, 1)
        if packets_read == 0:
            break

        if len(logs) > 0:
            for log in logs:
                print(f'[After seq {last_seq}]{log}')

        if sf.state[1] == last_seq:
            continue
        last_seq = sf.state[1]

        data: list[Any] = []

        for var_def in var_defs:
            data.append(sf.state[var_def[2]])

        writer.writerow(data)
    pass