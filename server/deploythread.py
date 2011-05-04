from threading import Thread
from paramiko import SSHClient, AutoAddPolicy
from scp import SCPClient
from channel import Listener
import tempfile
import os
import sys

sys.path.append('../commandlets/ride2ros')
from ride2ros import roslaunch_xml_for_file

class DeployThread(Thread):
    def __init__(self, host, user, password, project):
        Thread.__init__(self)
        self.host = host
        self.user = user
        self.password = password
        self.project = project
        self.remote_deploy_path = '~/ride_deploy'
        self.die = False

    def kill(self):
        self.die = True

    def run(self):
        try:
            # run a command and wait for it to finish
            def run(command):
                _, stdout, _ = ssh.exec_command(command)
                stdout.channel.recv_exit_status()

            # send the webapp a textual progress update
            listener = Listener()
            def log(text):
                listener.publish(('project', self.project.name, 'deploy', 'status'), { 'text': text })

            # prevent accidentally deploying on the server itself
            # (this doesn't prevent deploying to the server's public
            # IP address, so there's still a security problem here)
            if self.host in ['127.0.0.1', '0.0.0.0']:
                raise Exception('you\'re trying to deploy to the server!')

            # create temporary build file
            path = os.path.join(tempfile.mkdtemp(), 'deploy.launch')
            xml = roslaunch_xml_for_file(self.project.project_file_path, self.remote_deploy_path)
            open(path, 'w').write(xml)
            log('Created file: deploy.launch')

            # create a ssh session
            log('Connecting to %s@%s...' % (self.user, self.host))
            ssh = SSHClient()
            ssh.load_system_host_keys()
            ssh.set_missing_host_key_policy(AutoAddPolicy())
            ssh.connect(self.host, username=self.user, password=self.password)
            log('Connected to remote machine')

            # clean the deploy location
            run('rm -fr %s' % self.remote_deploy_path)
            run('mkdir -p %s' % self.remote_deploy_path)
            log('Cleaned deploy location')

            # copy the launch file over
            scp = SCPClient(ssh.get_transport())
            scp.put(path, remote_path=self.remote_deploy_path)
            log('Copied file: deploy.launch')

            # run the file
            command = 'cd %s && roslaunch deploy.launch' % self.remote_deploy_path
            channel = ssh.invoke_shell()
            channel.send(command + '\nexit\n')
            log('Ran command: roslaunch deploy.launch')

            # wait for the command to finish
            import time
            while not self.die:
                time.sleep(0.1)
        except:
            import traceback
            log('Exception:\n' + traceback.format_exc())

        ssh.close()
        log('Connection was closed')
