rm -r data/tmp
mkdir data/tmp
x=1; for i in data/*depth.jpg; do counter=$(printf %03d $x); ln "$i" data/tmp/depth_img"$counter".jpg; x=$(($x+1)); done
x=1; for i in data/*rgb.jpg; do counter=$(printf %03d $x); ln "$i" data/tmp/rgb_img"$counter".jpg; x=$(($x+1)); done

./ffmpeg -i data/tmp/depth_img%03d.jpg -r 10 -vcodec mpeg4 data/tmp/depth.avi

./ffmpeg -i data/tmp/rgb_img%03d.jpg -r 10 -vcodec mpeg4 data/tmp/rgb.avi

rm data/tmp/*.jpg
