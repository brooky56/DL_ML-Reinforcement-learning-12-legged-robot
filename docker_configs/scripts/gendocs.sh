#!/bin/bash

# get all packages
packages=$(find ~/catkin_ws/src -name "package.xml" | sed -e "s/^\/home\/app\/catkin_ws\/src\///" -e "s/\/package.xml$//")

index_tmpl='<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width"/>
    <meta http-equiv="Content-Type" content="text/html; charset=UTF-8"/>
    <title>Package list</title>
</head>
<body>
    <div style="text-align: center; font-family: Arial, sans-serif; font-size: 18px;">
        <h1 style="position: relative; color: #4fbbd6; margin-top: 0.2em;">Package list</h1>
        <h3 style="position: relative; color: #666666; margin-top: 0.2em;">Generated at {{.GenDate}}</h3>
        <ul style="position: relative; color: #666666; margin-top: 0.2em;">
            <!-- <li style="margin-top: 0.2em"><a href="{{.PackageName}}}/index.html">{{.PackageName}}</a></li> -->
            {{.PackageList}}
        </ul>
    </div>
</body>
</html>'

# iterating over packages
while IFS= read -r package; do
  echo "\e[95mGenerating docs for package $package\e[0m"

  # generating initial sphinx configs for each package
  sphinx-quickstart -q --project="$package" --author="StriRus team" -v "0.0.1" \
    --ext-autodoc --ext-doctest --ext-intersphinx --ext-todo --ext-coverage \
    --ext-viewcode --makefile --no-batchfile ~/catkin_ws/src/$package

  # generating documentation
  rosdoc_lite -o ~/catkin_ws/docs/$package ~/catkin_ws/src/$package

  # accumulating links to package indexes
  package_list="$package_list<li style=\"margin-top: 0\.2em\"><a href=\"$(echo $package | sed 's/\//\\\//g' )\/html\/index\.html\">$(echo $package | sed 's/\//\\\//g' )<\/a><\/li>"

done <<< "$packages"

echo "\e[95mGenerating index for packages\e[0m"

# placing the date of generation
idx=$(echo $index_tmpl | sed -e "s/{{\.GenDate}}/$(date '+%Y-%m-%dT%H:%M:%S')/g")

echo "\e[95mSubstituting the list of package indexes\e[0m"
# placing list of package indexes
idx=$(echo $idx | sed -e "s/{{\.PackageList}}/$package_list/g")

echo $idx > ~/catkin_ws/docs/index.html
