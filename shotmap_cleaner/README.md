# Shot Map Cleaner

## Installation

Use `uv` or `pip` to install:

```sh
uv sync
```

or

```sh
pip install -r requirements.txt
```

If you want to use a virtual environment with pip:

```sh
py -m venv venv

.\venv\Scripts\activate

pip install -r requirements.txt
```

## Usage:

Simply run main.py  (`uv run main.py` or `py main.py`).

Select either manual or automatic mode.

Automatic mode will prompt you for error bounds. These should likely be considerably smaller than the acceptable error margin for our fire control; this error will be added to that error in terms of the final amount of error allowed while shooting, as this error controls the *setpoint* rather than the actual position. You can use h/j/k/l to move your cursor left and right and to increment/decrement the hovered digit of the number you're entering.

Manual mode will allow you to manually pick points to keep/drop:

Select data points to keep (k/j for up and down and space to toggle selection). Press enter and the program will compare the map you've cherrypicked to the original map. It will print out a summary of the differences between the maps (itemwise subtraction for each datapoint from min to max distance at 1cm increments). If you're satisfied, answer `y` to the done prompt. Otherwise, answer `n` to select new points.

Once you've indicated that you're done, the program will serialize the new map to `ShotMapsCleaned.json` in the `deploy/constants/comp/` directory.
