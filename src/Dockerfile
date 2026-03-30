# FrikBotNex server build — matches Pi runtime (Wolfi/glibc)
FROM cgr.dev/chainguard/wolfi-base:latest AS server-builder
WORKDIR /src

RUN apk add --no-cache gcc make glibc-dev libstdc++-dev patch ca-certificates bash

# Build infrastructure
COPY src/build/ src/build/
COPY src/server/ src/server/

# Pre-fetched WinQuake (from CI prefetch step)
COPY src/build/tmp/WinQuake src/build/tmp/WinQuake

# NexQuake vendor (submodule — only build + bugfix + server needed)
COPY vendor/NexQuake/build/ vendor/NexQuake/build/
COPY vendor/NexQuake/bugfix/ vendor/NexQuake/bugfix/
COPY vendor/NexQuake/server/ vendor/NexQuake/server/

# Recast/Detour vendor
COPY vendor/recastnavigation/ vendor/recastnavigation/

ARG TARGETPLATFORM
RUN set -eu; \
	mkdir -p /out; \
	OUT=/out/nqserver PLATFORM="${TARGETPLATFORM:-}" bash src/build/build-server.sh

FROM scratch AS server-artifact
COPY --from=server-builder /out/nqserver /nqserver
