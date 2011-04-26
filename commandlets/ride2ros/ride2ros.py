from easyxml import EasyXML
import yaml

next_ids = {}

def new_name(prefix):
    next_ids[prefix] = next_ids.get(prefix, 0) + 1
    return prefix + str(next_ids[prefix])

def roslaunch_xml_for_file(path, package_path):
    data = yaml.safe_load(file(path))
    xml = EasyXML('launch')

    xml.machine(
        name='ride',
        address='localhost',
        default='true',
        ros_package_path='%s:$(env ROS_PACKAGE_PATH)' % package_path
    )

    # re-assign names to make sure they are valid and unique
    for node in data['nodes']:
        node['name'] = new_name('node')

    # remap topic names too
    id_to_input = {}
    for node in data['nodes']:
        for input in node['inputs']:
            id_to_input[input['id']] = input
    for node in data['nodes']:
        for output in node['outputs']:
            if output['connections']:
                output['remap'] = new_name('topic')
                for id in output['connections']:
                    id_to_input[id]['remap'] = output['remap']

    # generate node xml
    for node in data['nodes']:
        if 'exec' in node:
            xml.node(
                name=node['name'],
                pkg=node['pkg'],
                type=node['exec'],
                output='screen',
            )
            for input in node['inputs']:
                if 'remap' in input:
                    xml.node.remap(**{ 'from': input['name'], 'to': input['remap'] })
            for output in node['outputs']:
                if 'remap' in output:
                    xml.node.remap(**{ 'from': output['name'], 'to': output['remap'] })
        elif 'launch' in node:
            xml.include(file='$(find %s)/%s' % (node['pkg'], node['launch']))
        else:
            print 'warning: node "%s" has no launch configuration, and therefore will not be run when package is launched' % node['name']

    # generate xml string
    return str(xml)

if __name__ == '__main__':
    import sys
    if len(sys.argv) == 2:
        print roslaunch_xml_for_file('project.yaml', sys.argv[1])
    else:
        print 'usage: ride2ros DEPLOY_PATH'
