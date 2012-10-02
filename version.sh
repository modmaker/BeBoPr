#!/usr/bin/awk -f
BEGIN {
version=0;
build=0;
date="someday";
printf "/* This is a generated file, any changes will be overwritten */\n";
}
/MENDEL_VERSION/ { printf "#define MENDEL_VERSION\t%d\n", $3; }
/MENDEL_BUILD/   { printf "#define MENDEL_BUILD\t%d\n", $3+1; }
/MENDEL_DATE/    { "date -u +'%a %d-%m-%Y %H:%M UTC'" | getline date; printf "#define MENDEL_DATE\t\"%s\"\n", date; }
