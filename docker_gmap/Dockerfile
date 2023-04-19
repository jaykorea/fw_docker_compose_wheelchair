FROM freeway:rev.09_jay

CMD ["/bin/zsh", "-c", "XSOCK=/tmp/.X11-unix"]
CMD ["/bin/zsh", "-c", "XAUTH=/tmp/.docker.xauth"]
CMD ["/bin/zsh", "-c", "touch $XAUTH"]
CMD ["/bin/zsh", "-c", "xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -"]

