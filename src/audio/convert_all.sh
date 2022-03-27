mkdir enlarged
for i in *.wav; do ffmpeg -i "$i" -af "adelay=10000|10000" -acodec pcm_s16le -ar 48000 -ac 2 enlarged/"$i"; done
mkdir padded_normalized
ffmpeg-normalize enlarged/*.wav -of padded_normalized -ext wav -nt rms -pr
rm -rf enlarged
for i in padded_normalized/*.wav; do ffmpeg -i "$i" -ss 00:00:10.000 -acodec copy converted/$(basename -- "$i"); done
# rm -rf padded_normalized