#! /bin/bash

# Run the following if you run into even more problems:
# sudo -H pip install jupyterlab -U

poetry run python -m ipykernel install --user --name=`basename $VIRTUAL_ENV`
poetry run jupyter notebook
