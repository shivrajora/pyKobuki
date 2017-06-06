rm -r data/tmp
mkdir data/tmp
x=1; for i in data/*depth.png; do counter=$(printf %03d $x); ln "$i" data/tmp/depth_img"$counter".png; x=$(($x+1)); done
x=1; for i in data/*rgb.png; do counter=$(printf %03d $x); ln "$i" data/tmp/rgb_img"$counter".png; x=$(($x+1)); done

./ffmpeg -i data/tmp/depth_img%03d.png -r 10 -vcodec mpeg4 data/tmp/depth.avi

./ffmpeg -i data/tmp/rgb_img%03d.png -r 10 -vcodec mpeg4 data/tmp/rgb.avi

rm data/tmp/*.png
