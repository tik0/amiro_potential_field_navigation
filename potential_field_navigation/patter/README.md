# Patterns

## Video creation

Create videos uncompressed via `ffmpeg -i dot%04d.png -c:v huffyuv test.avi`.
Compressed (e.g. via h264) do not work well.
