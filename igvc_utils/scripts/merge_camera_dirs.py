import argparse
import os
import sys
import shutil

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

parser = argparse.ArgumentParser(description='Merge the contents of multiple directories in sequence in order to prevent file naming conflicts.')
parser.add_argument('-d','--directories', nargs='+', help='Absolute paths of directories to merge in sequence', required=True)
parser.add_argument('-o','--out_dir_path', nargs='?', type=str, help='Absolte path to output dir name. i.e. /PATH/TO/OUTPUT/DIR/DIR_NAME', required=False, default='out')
args = parser.parse_args()

def copy_rename(old_file_name, new_file_name):
        src_dir= os.curdir
        dst_dir= os.path.join(os.curdir , "subfolder")
        src_file = os.path.join(src_dir, old_file_name)
        shutil.copy(src_file,dst_dir)

        dst_file = os.path.join(dst_dir, old_file_name)
        new_dst_file_name = os.path.join(dst_dir, new_file_name)
        os.rename(dst_file, new_dst_file_name)


if __name__=="__main__":

    out_dir_path = args.out_dir_path

    for dir in args.directories:
        if not os.path.exists(dir):
            print(bcolors.FAIL + "Directory [" + dir + "] does not exist. Please check supplied path." + bcolors.ENDC)
            sys.exit()


    if not os.path.exists(out_dir_path):
        os.mkdir(out_dir_path)

    padding = len(str(sum([len(os.listdir(dir)) for dir in args.directories])))

    # cycle through directories
    i = 0
    for dir in args.directories:
        files = os.listdir(dir)
        print(bcolors.OKBLUE + "Merging " + str(len(files)) + " from " + dir)
        # cycle through files within directory
        for file in files:
            src_file = os.path.join(dir, file)
            shutil.copy(src_file, out_dir_path)

            dst_file = os.path.join(out_dir_path, file)
            extension = os.path.splitext(file)[1]
            new_dst_file_name = os.path.join(out_dir_path, format(i, '0' + str(padding)) + extension)
            os.rename(dst_file, new_dst_file_name)
            i+=1
