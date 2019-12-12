( cat includes.txt | while read dir; do ls $dir/*.h; done; ) | xargs ctags
