$(window).load(function() {
	var editor = $('iframe')[0].contentWindow.editor;
	$('iframe').focus();

	// tell the iframe about the project url
	var projectName = /^\/project\/(\w+)\/?$/.exec(location.pathname)[1];
	editor.setProjectName(projectName);

	// create the toolbar buttons
	var contents = [
		new Toolbar.Button('Settings', '/static/images/settings.png').floatRight(),
		new Toolbar.Button('Insert ROS Node', '/static/images/rosnode.png').click(function() {
			if (library.isVisible()) library.hide();
			else library.show();
			window.focus();
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

	// create the node library
	var library = new HUD(document.body, 'Node Library');
	var libraryJSON = null;
	library.setButtons([
		new HUD.DefaultButton('Insert', function() {
			library.hide();
			var json = libraryJSON.nodes[library.selectionIndex];
			json.x = 100;
			json.y = 100;
			editor.insertNodeFromLibrary(json);
		}),
		new HUD.Button('Cancel', function() {
			library.hide();
		})
	]);

	// populate the node library
	channel('workspace', 'library', 'response').subscribe(function(json) {
		var contents = [];
		for (var i = 0; i < json.nodes.length; i++) {
			var node = json.nodes[i];
			var name = node.name;
			var path = node.pkg + '/' + (node.exec || node.launch);
			contents.push(new HUD.Row('<div class="name">' + name + '</div><div class="path">' + path + '</div>'));
		}
		library.setContents(contents);
		libraryJSON = json;
	});
	channel('workspace', 'library', 'request').publish({});
});
