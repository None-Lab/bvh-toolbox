from src.convert import bvh2csv

files = ["./test/ex/PT_S104_1_FM_M_009.bvh", "./test/ex/PT_S104_1_FM_M_023.bvh"]

for file in files:
    returnVal = bvh2csv(bvh_path=files[0], dst_dirpath="test/out/", CPU_count=15)
    print(returnVal)