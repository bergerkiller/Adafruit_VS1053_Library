This is a commandline tool for recording OGG audio.

It can run in the following modes:
- `main` = (default) listen on local tcp port 3000 and write ogg data to receiving clients
- `main -` = write ogg data to stdout (bash redirect to write to file, etc.)
- `main stream` = write stdin to local tcp port 3000 (used it for some testing)

Recommended use with FFMPEG! You can use the -i tcp://127.0.0.0:3000 as input after launching.