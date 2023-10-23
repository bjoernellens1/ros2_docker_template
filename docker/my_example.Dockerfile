# explicitly use Debian for maximum cross-architecture compatibility
FROM debian:bullseye-slim AS hello-world

COPY ./docker .

CMD ["< ./hello.txt"]
