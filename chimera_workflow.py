import csv
import os

# Chimera libraries
import chimera
from chimera import runCommand

runCommand('open 1d86')
runCommand('display')
runCommand('~ribbon')

previous_position = [15, 15, 15]
counter = 1
while counter == 1:
    counter = 1
    try:
        with open('head_position.csv', 'r') as file:
            i = 0
            reader = csv.reader(file)
            info = [row for row in reader]
    except ValueError:
        continue

    try:
        os.remove("head_position.csv")
    except ValueError:
        print('no file')

    for row in info:
        i += 1
        c = 100
        if i >= 2:
            command = 'move {},{},{}; wait {}'.format(float(row[0]) * c - previous_position[0],
                                                      float(row[1]) * c - previous_position[1],
                                                      float(row[2]) * c - previous_position[2], 1)
            runCommand(command)

        previous_position = [float(row[0]) * c, float(row[1]) * c, float(row[2]) * c]
