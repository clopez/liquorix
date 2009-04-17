#!/usr/bin/env bash

BROKENOUT_DIR="/tmp/.zen-sources/broken-out"

# Fancy Print
print() {
	echo -e "\033[0m\033[1;32m* \033[0m$*"
}
# YOU FAIL
youfail() {
	echo -e "\033[0m\033[1;31m!!!!!\033[0m \033[0m\033[1;35m$*\033[0m \033[0m\033[1;31m!!!!!\033[0m"
}

if [[ ${1} == "-c" || ${1} == "--checkout" ]] ; then
	for a in $(git branch -a | sed 's/^[ \t]*//' | sed 's#^origin/HEAD##' | grep origin); do
		b=$(echo $a | cut -f 2 -d "/")
		if [[ $b != "master" ]]; then
			git branch $b $a
		fi
	done
elif [[ ${1} == "-n" || ${1} == "--new" ]] ; then
	for a in $(git branch -a | sed 's/^[ \t]*//' | sed 's#^origin/HEAD##' | grep origin); do
		b=$(echo $a | cut -f 2 -d "/")
		if [[ ${2} == "" ]]; then
			print "--new requires an argument"
			exit 0
		fi
		if [[ $b != "master" ]]; then
			git branch $b ${2}
		fi
	done
elif [[ ${1} == "-p" || ${1} == "--patch" ]] ; then
	if [[ -d ${BROKENOUT_DIR} ]]; then
		rm -rf ${BROKENOUT_DIR}
	fi
	mkdir -p ${BROKENOUT_DIR}
	git checkout master
        for a in $(git branch | sed 's/^[ \t]*//;/^*/d'); do
                git diff ${2} $a > ${BROKENOUT_DIR}/$a.patch
        done
elif [[ ${1} == "-d" || ${1} == "--delete" ]] ; then

	git checkout master
	for a in $(git branch | sed 's/^[ \t]*//;/^*/d'); do
		git branch -D $a
	done
elif [[ ${1} == "-pa" || ${1} == "--push-all" ]] ; then
        git checkout master
        for a in $(git branch | sed 's/^[ \t]*//;/^*/d'); do
                git push origin ${a}
        done
elif [[ ${1} == "-fpa" || ${1} == "--force-push-all" ]] ; then
        git checkout master
        for a in $(git branch | sed 's/^[ \t]*//;/^*/d'); do
                git push -f origin ${a}
        done
elif [[ ${1} == "-b" && -n ${2} || ${1} == "--branch" && -n ${2} ]] ; then
	git checkout -b "${2}" origin/"${2}"
elif [[ ${1} == "-h" || ${1} == "--help" || ${1} == "" ]] ; then
	print "USAGE: ${0} [OPTION]"
	print ""
	print "Options:"
	print "-c	--checkout 	 Checkout all the branches"
	print "-n	--new		 Checkout all new branches"
	print "-b	--branch	 Checkout a branch"
	print "-d	--delete	 Delete all the branches"
	print "-p	--patch		 Make a patch of each branch"
	print "-pa	--push-all	 Push all the branches"
	print "-fpa	--force-push-all Force push all the branches"
	print "-h	--help		 Print this help message"
else
	youfail "Invalid Option '${1}'"
fi
