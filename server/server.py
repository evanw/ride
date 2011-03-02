#!/usr/bin/env python

import os
from channel import subscribe, Channel as ch
from project_manager import ProjectManager

project_manager = ProjectManager()
project_manager.set_workspace('~/ros_ide/workspace')

@subscribe('workspace', 'list')
def workspace_list(data):
    print 'workspace list request:', repr(data)
    ch('workspace', 'list').publish(project_manager.get_projects('json'))

# let the channel server do its thing
import time; time.sleep(999999)
