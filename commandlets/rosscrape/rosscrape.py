#!/usr/bin/env python

import sys
import time
import re
import urllib2

from optparse import OptionParser, OptionGroup

from yaml import dump as yaml_dump
try:
    from yaml import CDumper as Dumper
except ImportError:
    from yaml import Dumper

import sqlite3

verbose = False

# This determines how many items are processed at each "stage" of the scraping.
# A width of 3 means that three repos will be processed, and up to 3 of the
# packages in those repos will be processed.  Setting WIDTH to a non-positive
# value causes _ALL_ repos and packages to be processed.  THIS IS SLOW AND
# BANDWIDTH INTENSIVE - use infrequently.
WIDTH = 0

# This function ensures we don't flood the server with requests when scraping.
def fetch(url):
    time.sleep(3.0)
    return urllib2.urlopen(url)

# This scrapes the package's details page for more detailed package info.
def scrape_pkg_info(pkg, num, numpkgs):
    if WIDTH > 0 and WIDTH < numpkgs:
        numpkgs = WIDTH
    if verbose: sys.stdout.write('    Fetching data for %s... [%d of %d]' % (pkg, num, numpkgs)); sys.stdout.flush()
    pkg_info_data = fetch('http://www.ros.org/browse/details.php?name=' + pkg).read()
    
    pkg_info = dict()
    if verbose: sys.stdout.write('\r    Scraping data for %s... [%d of %d]' % (pkg, num, numpkgs)); sys.stdout.flush()
    match = re.search("""
                      Author\(s\):</b>([^<]+)</p>
                      .+?
                      License:</b>([^<]+)</p>
                      .+?
                      href="([^"]+)"
                      .+?
                      href="([^"]+)"
                      """, pkg_info_data, re.VERBOSE | re.DOTALL | re.UNICODE)
    if not match:
        if verbose: sys.stdout.write('\n')
        print('    Error getting package data for "%s"!' % pkg)
        return pkg_info
    
    groups = match.groups()
    pkg_info['author'] = unicode(groups[0].strip(), 'latin')
    pkg_info['license'] = groups[1].strip()
    pkg_info['website'] = groups[2].strip()
    pkg_info['source'] = groups[3].strip()
    
    if verbose: sys.stdout.write('\r    Scraping dependencies for %s... [%d of %d]' % (pkg, num, numpkgs)); sys.stdout.flush()
    deps = []
    for match2 in re.finditer('name=([^"]+)"', pkg_info_data[match.end():]):
        deps.append(match2.group(1))
        match = match2
    pkg_info['deps'] = deps
    
    if verbose: sys.stdout.write('\r    Scraping description for %s... [%d of %d]' % (pkg, num, numpkgs)); sys.stdout.flush()
    match3 = re.search('Description:</b>(.+?)</p>\s*<hr />' , pkg_info_data[match.end():], re.DOTALL)
    
    if not match3:
        if verbose: sys.stdout.write('\n')
        print('    Error getting package description for "%s"!' % pkg)
        return pkg_info
    pkg_info['desc'] = match3.group(1).strip()
    if verbose: print('\r    Scraped info for %s... [%d of %d]         ' % (pkg, num, numpkgs))
    return pkg_info

# This scrapes the list of packages for the given repository.  It also
# downloads the short descriptions, as they are easier to scrape here
# than on the package's page.
def scrape_pkgs(repo, num, numrepos):
    if WIDTH > 0 and WIDTH < numrepos:
        numrepos = WIDTH
    if verbose: print('Scraping package list for %s... [%d of %d]' % (repo, num, numrepos))
    repo_pkgs_data = fetch('http://www.ros.org/browse/repo.php?repo_host=' + repo).read()
    repo_pkgs = dict()
    count = 0
    matches = re.findall('\?name=([^"]+)">.+?</a></td><td>([^<]+)<', repo_pkgs_data, re.DOTALL)
    for pkgnum in range(len(matches)):
        if WIDTH > 0 and count >= WIDTH:
            break
        pkg = matches[pkgnum][0]
        repo_pkgs[pkg] = scrape_pkg_info(pkg, pkgnum + 1, len(matches))
        repo_pkgs[pkg]['shortdesc'] = matches[pkgnum][1].strip()
        count += 1
    return repo_pkgs

# This scrapes the list of repositories off the ROS website.  It returns the
# data in a "yaml-friendly" format.
def scrape_repos():
    # Grab the source of the repo list page.
    if verbose: print('Scraping repository list...')
    ros_repos_data = fetch('http://www.ros.org/browse/repo_list.php').read()
    ros_repos = dict()
    # Find lines listing repositories, and scrape the data off their pages.
    count = 0
    matches = re.findall('repo_host=([^"]+)"', ros_repos_data)
    for reponum in range(len(matches)):
    #for match in re.finditer('repo_host=([^"]+)"', ros_repos_data):
        if WIDTH > 0 and count >= WIDTH:
            break
        ros_repos[matches[reponum]] = scrape_pkgs(matches[reponum], reponum + 1, len(matches))
        count += 1
    
    return ros_repos

# Scrape all of the data on repositories, convert it to yaml, and print it out.
def main(options):
    global verbose, WIDTH
    if options:
        verbose = options.verbose
        WIDTH = options.width
    scrape_data = scrape_repos()
    if options.yaml:
        if verbose: print('Writing scraped data to rosrepos.yaml...')
        # Open the output file.
        output = open('rosrepos.yaml', 'w')
        # Convert the data to yaml and write it to the output file.
        yaml_data = yaml_dump(scrape_data, default_flow_style=False)
        output.write(yaml_data)
    if options.sqlite:
        if verbose: print('Writing scraped data to rosrepos.sqlite...')
        # Create a sqlite database - rosrepos.sqlite.
        conn = sqlite3.connect('rosrepos.sqlite')
        c = conn.cursor()
        # Delete any old data in the table.
        try:
            c.execute('drop table packages')
        except sqlite3.OperationalError:
            pass
        # Create the table for the data.
        c.execute('''create table packages
                    (pkg text, repo text, author text,
                    deps text, desc text, license text,
                    shortdesc text, source text, website text)''')
        
        # Enter the data into the table.
        for repo in scrape_data.keys():
            for pkg in scrape_data[repo].keys():
                data = scrape_data[repo][pkg]
                c.execute('insert into packages values (?,?,?,?,?,?,?,?,?)', \
                            (pkg, repo, data['author'], ' '.join(data['deps']), data['desc'], \
                            data['license'], data['shortdesc'], data['source'], data['website']))
        
        # Commit the changes to the table, and close everything up.
        conn.commit()
        c.close()
        conn.close()

if __name__ == "__main__":
    parser = OptionParser(version='rosscrape beta 0.7b')
    parser.add_option('-v', '--verbose', dest='verbose', action='store_true', \
                      help='Enable verbose output.')
    parser.add_option('-w', '--width', dest='width', action='store', type='int', \
                      help='Set width of scraping.  (Number of packages scraped is at most width^2.)')
    outgroup = OptionGroup(parser, 'Output Options')
    outgroup.add_option('-y', '--yaml', dest='yaml', action='store_true', \
                      help='Output data as yaml.')
    outgroup.add_option('-s', '--sqlite', dest='sqlite', action='store_true', \
                      help='Output data as sqlite database.')
    parser.add_option_group(outgroup)
    
    (options, args) = parser.parse_args()
    
    if not (options.yaml or options.sqlite):
        print('No output format selected; exiting...')
        sys.exit(0)
    
    main(options)

