#!/usr/bin/env python

import time
import re
import urllib2

from yaml import dump as yaml_dump
try:
    from yaml import CDumper as Dumper
except ImportError:
    from yaml import Dumper

# This determines how many items are processed at each "stage" of the scraping.
# A width of 3 means that three repos will be processed, and up to 3 of the
# packages in those repos will be processed.  Setting WIDTH to a non-positive
# value causes _ALL_ repos and packages to be processed.  THIS IS SLOW AND
# BANDWIDTH INTENSIVE - use infrequently.
WIDTH = 2

# This function ensures we don't flood the server with requests when scraping.
def fetch(url):
    time.sleep(1.5)
    return urllib2.urlopen(url)

# This scrapes the package's details page for more detailed package info.
def scrape_pkg_info(pkg):
    pkg_info_data = fetch('http://www.ros.org/browse/details.php?name=' + pkg).read()
    pkg_info = dict()
    match = re.search("""
                      Author\(s\):</b>([^<]+)</p>
                      .+?
                      License:</b>([^<]+)</p>
                      .+?
                      href="([^"]+)"
                      .+?
                      href="([^"]+)"
                      """, pkg_info_data, re.VERBOSE | re.DOTALL)
    if not match:
        print('error getting package data for "' + pkg + '"!')
        return pkg_info
    
    groups = match.groups()
    pkg_info['author'] = groups[0].strip()
    pkg_info['license'] = groups[1].strip()
    pkg_info['website'] = groups[2].strip()
    pkg_info['source'] = groups[3].strip()
    
    deps = []
    for match2 in re.finditer('name=([^"]+)"', pkg_info_data[match.end():]):
        deps.append(match2.group(1))
        match = match2
    pkg_info['deps'] = deps
    
    match3 = re.search('Description:</b>(.+?)</p>\s*<hr />' , pkg_info_data[match.end():], re.DOTALL)
    
    if not match3:
        print('error getting package description for "' + pkg + '"!')
        return pkg_info
    pkg_info['desc'] = match3.group(1).strip()
    return pkg_info

# This scrapes the list of packages for the given repository.  It also
# downloads the short descriptions, as they are easier to scrape here
# than on the package's page.
def scrape_pkgs(repo):
    repo_pkgs_data = fetch('http://www.ros.org/browse/repo.php?repo_host=' + repo).read()
    repo_pkgs = dict()
    count = 0
    for match in re.finditer('\?name=([^"]+)">.+?</a></td><td>([^<]+)<', repo_pkgs_data, re.DOTALL):
        if WIDTH > 0 and count >= WIDTH:
            break
        pkg = match.group(1)
        repo_pkgs[pkg] = scrape_pkg_info(pkg)
        repo_pkgs[pkg]['shortdesc'] = match.group(2).strip()
        count += 1
    return repo_pkgs

# This scrapes the list of repositories off the ROS website.  It returns the
# data in a "yaml-friendly" format.
def scrape_repos():
    # Grab the source of the repo list page.
    ros_repos_data = fetch('http://www.ros.org/browse/repo_list.php').read()
    ros_repos = dict()
    # Find lines listing repositories, and scrape the data off their pages.
    count = 0
    for match in re.finditer('repo_host=([^"]+)"', ros_repos_data):
        if WIDTH > 0 and count >= WIDTH:
            break
        ros_repos[match.group(1)] = scrape_pkgs(match.group(1))
        count += 1
    
    return ros_repos

# Scrape all of the data on repositories, convert it to yaml, and print it out.
def main():
    output = open('rosrepos.yaml', 'w')
    scrape_data = scrape_repos()
    yaml_data = yaml_dump(scrape_data, default_flow_style=False)
    output.write(yaml_data)

if __name__ == "__main__":
    main()
