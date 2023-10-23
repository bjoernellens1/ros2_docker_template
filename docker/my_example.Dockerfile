# explicitly use Debian for maximum cross-architecture compatibility
FROM debian:bullseye-slim AS hello-world

RUN set -eux; \
	apt-get update; \
	apt-get install -y --no-install-recommends \
		ca-certificates \
		gnupg \
		wget \
		\
		gcc \
		libc6-dev \
		make \
		\
		libc6-dev-arm64-cross \
		libc6-dev-armel-cross \
		libc6-dev-armhf-cross \
		libc6-dev-i386-cross \
		libc6-dev-mips64el-cross \
		libc6-dev-ppc64el-cross \
		libc6-dev-riscv64-cross \
		libc6-dev-s390x-cross \
		\
		gcc-aarch64-linux-gnu \
		gcc-arm-linux-gnueabi \
		gcc-arm-linux-gnueabihf \
		gcc-i686-linux-gnu \
		gcc-mips64el-linux-gnuabi64 \
		gcc-powerpc64le-linux-gnu \
		gcc-riscv64-linux-gnu \
		gcc-s390x-linux-gnu \
		\
		arch-test \
		file \
	; \
	rm -rf /var/lib/apt/lists/*

# https://musl.libc.org/releases.html
ENV MUSL_VERSION 1.2.4
RUN set -eux; \
	wget -O musl.tgz.asc "https://musl.libc.org/releases/musl-$MUSL_VERSION.tar.gz.asc"; \
	wget -O musl.tgz "https://musl.libc.org/releases/musl-$MUSL_VERSION.tar.gz"; \
	\
	export GNUPGHOME="$(mktemp -d)"; \
	gpg --batch --keyserver keyserver.ubuntu.com --recv-keys '836489290BB6B70F99FFDA0556BCDB593020450F'; \
	gpg --batch --verify musl.tgz.asc musl.tgz; \
	gpgconf --kill all; \
	rm -rf "$GNUPGHOME" musl.tgz.asc; \
	\
	mkdir /usr/local/src/musl; \
	tar --extract --file musl.tgz --directory /usr/local/src/musl --strip-components 1; \
	rm musl.tgz

WORKDIR /usr/src/hello
COPY . .


RUN find \( -name 'hello' -or -name 'hello.txt' \) -exec file '{}' + -exec ls -lh '{}' +

CMD ["./amd64/hello-world/hello"]
