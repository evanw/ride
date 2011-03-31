$(window).load(function() {
	var editor = $('iframe')[0].contentWindow.editor;
	$('iframe').focus();

	var projectName = /^\/project\/(\w+)\/?$/.exec(location.pathname)[1];
	editor.setProjectName(projectName);

	var contents = [
		new Toolbar.Button('Settings', '/static/images/settings.png').floatRight(),
		new Toolbar.Button('ROS Node', '/static/images/rosnode.png').click(function() {
			editor.insertNodeFromLibrary({
				name: 'new_ros_node',
				x: 50,
				y: 50,
				id: 0,
				inputs: [{name:'in'}],
				outputs: [{name:'out'}]
			});
		}),
		new Toolbar.Button('Python Node', '/static/images/pythonnode.png').click(function() {
			editor.insertNodeFromLibrary({
				name: 'new_python_node',
				x: 50,
				y: 50,
				id: 0,
				inputs: [{name:'in'}],
				outputs: [{name:'out'}]
			});
		}),
		new Toolbar.Button('Run', '/static/images/run.png'),
		new Toolbar.Button('Stop', '/static/images/stop.png').setEnabled(false)
	];

	var toolbar = new Toolbar();
	toolbar.setContents(contents);

	function resize() {
		$('.editor').css({ top: toolbar.height() + 'px' });
	}
	$(window).resize(resize);
	resize();
});
