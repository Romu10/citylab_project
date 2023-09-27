#!/bin/bash

echo "CONFIGURING GIT LOGS"
git config --global user.name "Romu10"
git config --global user.email "romulobryanp@gmail.com"

name_git=$(git config --global user.name)
email_git=$(git config --global user.email)

if [ -z "$name_git" ] && [ -z "$email_git" ]; then
    echo "User name and email is not configurated."
else 
    echo "User name configurated is: $name_git"
    echo "User email configurated is: $email_git"
fi 

echo "DONE"

#Git password: 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