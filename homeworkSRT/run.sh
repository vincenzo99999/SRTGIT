#!/bin/bash

# Esegui controller sul core 0
gnome-terminal -- bash -c "taskset --cpu-list 0 ./controller"
# Esegui backup-controller sul core 1
gnome-terminal -- bash -c "taskset --cpu-list 1 ./backup-controller"

# Esegui plant sul core 0
gnome-terminal -- bash -c "taskset --cpu-list 0 ./plant"

# Esegui monitor sul core 0
#gnome-terminal -- bash -c "taskset --cpu-list 0 ./monitor"
gnome-terminal -- ./monitor