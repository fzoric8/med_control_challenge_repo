import csv
import os

class DataHandler:
    """
        Saves simulation data to a csv file
    """

    def __init__(self, file_name="data_table"):

        os.chdir("crobots_med_control/data")