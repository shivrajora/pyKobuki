D1=$1
echo $D1"tmp"
rm -r $D1"tmp"
mkdir $D1"tmp"
x=1; for i in $D1*depth.jpg; do counter=$(printf %03d $x); ln "$i" $D1"tmp/depth_img""$counter".jpg; x=$(($x+1)); done
x=1; for i in $D1*rgb.jpg; do counter=$(printf %03d $x); ln "$i" $D1"tmp/rgb_img""$counter".jpg; x=$(($x+1)); done

./ffmpeg -i $D1"tmp"/depth_img%03d.jpg -r 10 -vcodec mpeg4 $D1"tmp/depth.avi"

./ffmpeg -i $D1"tmp"/rgb_img%03d.jpg -r 10 -vcodec mpeg4 $D1"tmp/rgb.avi"

rm $D1"tmp"/*.jpg
