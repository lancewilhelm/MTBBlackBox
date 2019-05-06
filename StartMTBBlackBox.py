import MTBBlackBox
import sys
import argparse

# Allows the user to start the black box program with arguments
parser = argparse.ArgumentParser(description="Starts the MTB Black Box")
parser.add_argument('-o', type=str, default="n", help="whether or not to output live data to console (y, default = n)")
parser.add_argument('-d', type=str, default="n", help="debug mode (y, default = n). Allows trial on computer without sensor ")
args = parser.parse_args()
# all options should have defaults so we don't need to provide them when calling the script

print("Starting the MTB Black Box with arguments"+str(args))
MTBBlackBox.main(args)
