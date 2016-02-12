#!/usr/bin/python3

import os
import os.path as osp
import argparse as ap
import numpy as np
import re



if __name__ == "__main__":
    parser = ap.ArgumentParser(description='Delete all right frame files in folder based on which left frame files remain.')

    parser.add_argument("-f", "--folder", help="Folder to work in", 
                        required=False, default= "./")
    parser.add_argument("-s", "--save_frame_numbers", help="Save an array containing frame numbers of the remaining pairs.",
                        action="store_true",default = False)
    parser.add_argument("-p", "--prefixes", help="custom prefixes for the frames from left and right camera respectively",
                        required=False, nargs=2, default = None)
    
    args = parser.parse_args()
    files = [f for f in os.listdir(args.folder) if osp.isfile(osp.join(args.folder,f)) and f.endswith(".png")]
    files.sort()
    
    if(args.prefixes == None):
        rfiles = [f for f in files if "r_" in f or "R" in f]
        lfiles = [f for f in files if "l_" in f or "L" in f]
        rn_search = rfiles
        ln_search = lfiles
    else:
        rfiles = [f for f in files if f.startswith(args.prefixes[0])]
        lfiles = [f for f in files if f.startswith(args.prefixes[1])]
        len_l_prefix = len(args.prefixes[0])
        len_r_prefix = len(args.prefixes[1])
        rn_search = [f[len_l_prefix:] for f in rfiles]
        ln_search = [f[len_r_prefix:] for f in lfiles]
        
    num_pattern = re.compile("\d{4}\d?")
    rnums = [int(re.findall(num_pattern,f)[0]) for f in rn_search]
    lnums = [int(re.findall(num_pattern,f)[0]) for f in ln_search]
    rdict = {}
    ldict = {}
    
    for ix_right in range(len(rnums)):
        rfile = rfiles[ix_right]
        rnum = rnums[ix_right]
        rdict[rnum] = rfile
    for ix_left in range(len(lnums)):
        lfile = lfiles[ix_left]
        lnum = lnums[ix_left]
        ldict[lnum] = lfile
        
    count_r_removed = 0
    count_l_removed = 0
    #filter right based on left
    for rnum, rfile in rdict.items():
        if(not rnum in ldict):
            os.remove(osp.join(args.folder,rfile))
            count_r_removed += 1
    #filter left based on right
    for lnum, lfile in ldict.items():
        if(not lnum in rdict):
            os.remove(osp.join(args.folder,lfile))
            count_l_removed += 1

    print("removed {0:d} files ({1:d} of {3:d} for left and {2:d} of {4:d} for right)"\
        .format(count_l_removed + count_r_removed, 
                count_l_removed, count_r_removed,
                len(lnums), len(rnums)))
    
    if(args.save_frame_numbers):
        rnums_arr = np.array(rnums)
        np.savez_compressed(osp.join(args.folder,"frame_numbers.npz"),frame_numbers=rnums_arr)