#!/bin/bash

echo -e "\n" >> log
echo -e "\n" >> log
echo -e "\n" >> log
date >> log
for i in {0..200..1}
do
    ./a.out >> log
done
