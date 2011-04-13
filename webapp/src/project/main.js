$(window).load(function() {
	var editor = $('iframe')[0].contentWindow.editor;
	$('iframe').focus();

	// tell the iframe about the project url
	var projectName = /^\/project\/(\w+)\/?$/.exec(location.pathname)[1];
	editor.setProjectName(projectName);

	// create the toolbar buttons
	var libraryVisible = false;
	var contents = [
		new Toolbar.Button('Settings', '/static/images/settings.png').floatRight(),
		new Toolbar.Button('ROS Node', '/static/images/rosnode.png').click(function() {
			libraryVisible = !libraryVisible;
			if (libraryVisible) $('.library').stop().show().css({ opacity: 0 }).animate({ opacity: 1 });
			else $('.library').stop().fadeOut();
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

	// add the buttons to a toolbar
	var toolbar = new Toolbar();
	toolbar.setContents(contents);

	// position the editor below the toolbar
	function resize() {
		$('.editor').css({ top: toolbar.height() + 'px' });
	}
	$(window).resize(resize);
	resize();

	// populate the node library
	channel('workspace', 'library', 'response').subscribe(function(json) {
		var html = '';
		for (var i = 0; i < json.nodes.length; i++) {
			var node = json.nodes[i];
			var name = node.name;
			var path = node.pkg + '/' + (node.exec || node.launch);
			html += '<div class="' + (i == 1 ? 'selected ' : '') + 'node"><div class="name">' + name + '</div><div class="path">' + path + '</div></div>';
		}
		$('.library .nodelist').html(html);
	});
	channel('workspace', 'library', 'request').publish({});
});
