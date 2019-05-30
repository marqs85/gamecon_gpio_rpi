#/usr/bin/make
SRC = $(DESTDIR)/usr/src
SHARE = $(DESTDIR)/usr/share/$(NAME)-dkms
DOCS = $(DESTDIR)/usr/share/doc/$(NAME)

all:

clean:

install:

#source tree (remove mkdeb and README)
ifeq ("$(wildcard $(NAME)-$(VERSION))", "$(NAME)-$(VERSION)")
	install -d "$(SRC)"
	cp -a $(NAME)-$(VERSION) $(SRC)
	rm -r $(SRC)/$(NAME)-$(VERSION)/$(NAME)-dkms-mkdeb
	rm $(SRC)/$(NAME)-$(VERSION)/README
	chmod 644 -R "$(SRC)/$(NAME)-$(VERSION)"
endif

#tarball, possibly with binaries
ifeq ("$(wildcard $(NAME)-$(VERSION).dkms.tar.gz)", "$(NAME)-$(VERSION).dkms.tar.gz")
	install -d "$(SHARE)"
	install -m 644 $(NAME)-$(VERSION).dkms.tar.gz "$(SHARE)"
endif

#postinst, only if we are supporting legacy mode
ifeq ("$(wildcard common.postinst)", "common.postinst")
	install -d "$(SHARE)"
	install -m 755 $(PREFIX)/usr/lib/dkms/common.postinst $(SHARE)/postinst
endif

#document dir
	install -d "$(DOCS)"
	install -m 644 $(NAME)-$(VERSION)/README $(DOCS)/README
