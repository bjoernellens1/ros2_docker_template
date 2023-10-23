# explicitly use Debian for maximum cross-architecture compatibility
FROM debian:bullseye-slim AS cps-hello-world

COPY . .

CMD ["< ./hello.txt"]
