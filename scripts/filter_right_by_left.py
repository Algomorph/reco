#!/usr/bin/python

import os
import os.path as osp
import argparse as ap
import re

parser = ap.ArgumentParser(description='Delete all right frame files in folder based on which left frame files remain.')

parser.add_argument("-f", "--folder", help="Folder to work in", 
                    required=False, default= ["./"])

if __name__ == "__main__":
    args = parser.parse_args()
    files = [f for f in os.listdir(args.folder) if osp.isfile(osp.join(args.folder,f)) and f.endswith(".png")]
    files.sort()
    rfiles = [f for f in files if "r" in f]
    lfiles = [f for f in files if "l" in f]
    num_pattern = re.compile("\d+")
    rnums = [int(re.findall(num_pattern,f)[0]) for f in rfiles]
    lnums = [int(re.findall(num_pattern,f)[0]) for f in lfiles]
    
    ix_left = 0
    for ix_right in xrange(len(rnums)):
        rfile = rfiles[ix_right]
        rnum = rnums[ix_right]
        if ix_left >= len(lfiles) or lnums[ix_left] != rnum:
            os.remove(args.folder + osp.sep + rfile)
        else:
            ix_left += 1