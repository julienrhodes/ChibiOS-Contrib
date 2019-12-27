( cat includes.txt | while read dir; do ls $dir/*.{h,c}; done; ) | xargs ctags
