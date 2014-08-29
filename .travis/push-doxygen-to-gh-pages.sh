#!/bin/bash

# Example taken from 
#   https://github.com/ReadyTalk/swt-bling/blob/master/.utility/push-javadoc-to-gh-pages.sh
#   http://benlimmer.com/2013/12/26/automatically-publish-javadoc-to-gh-pages-with-travis-ci/

if [ "$TRAVIS_REPO_SLUG" == "robotology/icub-tactilegrasp" ] && [ "$TRAVIS_PULL_REQUEST" == "false" ] && [ "$TRAVIS_BRANCH" == "master" ] && [ "$TRAVIS_BUILD_TYPE" == "Release" ]; then

    echo -e "Publishing doxygen. \n"

    cd $PROJ_ROOT
    cp -R doc $HOME/doc-latest

    cd $HOME
    git config --global user.email "travis@travis-ci.org"
    git config --global user.name "travis-ci"
    git clone --quiet --branch=gh-pages https://${GH_TOKEN}@github.com/robotology/icub-tactilegrasp gh-pages > /dev/null

    cd gh-pages
    git rm -rf doc
    mkdir -p doc/
    cp -Rf $HOME/doc-latest/html ./doc/html
    git add -f .
    git commit -m "[GH-PAGES] [DOC] Updated doxygen on successful travis build $TRAVIS_BUILD_NUMBER auto-pushed to gh-pages"
    git push -fq origin gh-pages

    echo -e "Published doxygen to gh-pages. \n"

fi
