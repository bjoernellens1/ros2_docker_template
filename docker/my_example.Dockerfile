# explicitly use Debian for maximum cross-architecture compatibility
FROM debian:bullseye-slim AS hello-world

WORKDIR /APP

COPY ./docker .

CMD ["cat /APP/hello.txt"]
