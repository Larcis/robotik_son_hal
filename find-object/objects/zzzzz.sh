dir="/home/larcis/rtg_ws/src/find-object/objects/"
echo -n "string names[] = {"
for f in "$dir"*; do
  echo -n "\"${f:${#dir}:-4}\","
done
echo "};"
