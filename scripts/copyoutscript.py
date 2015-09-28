import os
import shutil

new_directory = str(raw_input('new directory name: '))

relevant_files = ['CMakeCache.txt', 'CMakeLists.txt']

for root, dirs, files in os.walk('.'):
 useful_files = set(relevant_files).intersection(set(files))
 if useful_files:
  for name in useful_files:
   sourcefile = os.path.join(root,name)
   print sourcefile
   try:
    os.makedirs(os.path.join(new_directory, root))
   except:
    pass
   shutil.copyfile(sourcefile, os.path.join(new_directory, root, name))

print 'success'
