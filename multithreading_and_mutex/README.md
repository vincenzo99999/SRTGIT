# Bit Recognizer

This is a simple real-time C program using POSIX threads to recognize a sequence of bits.

## How It Works

- Three real-time threads (generators) toggle bits at different intervals.
- A recognizer thread checks for the sequence: 000 → 011 → 110 → 101.
- A buddy thread (non-real-time) prints the current bit values and if the sequence was found.

## Run

Compile and run with:

```bash
gcc -o bitRecognizer bitRecognizer.c -lpthread
./bitRecognizer
```

Press `q` + Enter to quit.

## Files

- `bitRecognizer.c`: Main source code.

## To Do

- [ ] Make the bit sequence configurable.
- [ ] Save results to a file.
