function text2html(text) {
	return ('' + text).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/\n/g, '<br>').replace(/  /g, '&nbsp; ');
}

$(window).load(function() {
	var editor = $('iframe')[0].contentWindow.editor;
	$('iframe').focus();

	// tell the iframe about the project url
	var projectName = /^\/project\/(\w+)\/?$/.exec(location.pathname)[1];
	editor.setProjectName(projectName);

	// create the deploy status window
	var deployStatus = new HUD(document.body, 'Deploy Status');
	deployStatus.setButtons([
		new HUD.DefaultButton('Close', function() {
			deployStatus.hide();
		})
	]);
	var totalStatus = '';

	// create the toolbar buttons
	var contents = [
		new Toolbar.Button('Settings', '/static/images/settings.png').floatRight(),
		new Toolbar.Button('Insert ROS Node', '/static/images/rosnode.png').click(function() {
			if (library.isVisible()) library.hide();
			else library.show();
			window.focus();
		}),
		new Toolbar.Button('Run', '/static/images/run.png').click(function() {
			channel('project', projectName, 'deploy', 'run').publish({});
			deployStatus.show();
			totalStatus = 'Loading...';
			deployStatus.setContents([
				new HUD.Box(totalStatus)
			]);
		}),
		new Toolbar.Button('Stop', '/static/images/stop.png').click(function() {
			channel('project', projectName, 'deploy', 'stop').publish({});
			// deployStatus.hide();
		})
	];
	channel('project', projectName, 'deploy', 'status').subscribe(function(json) {
		totalStatus += text2html('\n' + json['text']);
		deployStatus.setContents([
			new HUD.Box(totalStatus)
		]);
	});

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
