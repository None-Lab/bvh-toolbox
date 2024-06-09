from src.convert import bvh2csv
from glob import glob

files = sorted(glob("RAW_Motion_Data/*.bvh"))

for file in files:
    returnVal = bvh2csv(bvh_path=file, dst_dirpath="Processed_Motion_Data/", 
                        export_rotation=False, export_position=True, export_hierarchy=False)
    print(returnVal)

