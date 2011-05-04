function text2html(text) {
	return ('' + text).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/\n/g, '<br>').replace(/  /g, '&nbsp; ');
}

var robot_ip = '127.0.0.1';
var username = 'obot';
var password = 'obot';

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

	var settings = new HUD(document.body, 'Settings');
	settings.setContents([
		new HUD.Textbox('Robot IP address', robot_ip, function(textbox) {
			robot_ip = textbox.text;
		}),
		new HUD.Textbox('Username', username, function(textbox) {
			username = textbox.text;
		}),
		new HUD.Textbox('Password', password, function(textbox) {
			password = textbox.text;
		}),
		new HUD.Box('Note: password is currently sent unencrypted!')
	]);
	settings.setButtons([
		new HUD.DefaultButton('Close', function() {
			settings.hide();
		})
	]);

	// create the toolbar buttons
	var contents = [
		new Toolbar.Button('Settings', '/static/images/settings.png').floatRight().click(function() {
			settings.show();
			window.focus();
		}),
		new Toolbar.Button('Insert ROS Node', '/static/images/rosnode.png').click(function() {
			if (library.isVisible()) library.hide();
			else library.show();
			window.focus();
		}),
		new Toolbar.Button('Run', '/static/images/run.png').click(function() {
			channel('project', projectName, 'deploy', 'run').publish({
				ip: robot_ip,
				user: username,
				pass: password
			});
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
	var libraryJSON = { nodes: [] };
	library.setButtons([
		new HUD.DefaultButton('Insert', function() {
			library.hide();
			var json = libraryJSON.nodes[library.selectionIndex];
			json.x = 100;
			json.y = 100;
			editor.insertNodeFromLibrary(json);
		}),
		new HUD.Button('New Node', function() {
			newNode.show();
		}),
		new HUD.Button('Cancel', function() {
			library.hide();
		})
	]);

	var newNode = new HUD(document.body, 'New Node');
	var newNodeName = '';
	var newNodePackage = '';
	var newNodeExecutable = '';
	var newNodeInputs = '';
	var newNodeOutputs = '';
	newNode.setContents([
		new HUD.Textbox('Name', newNodeName, function(textbox) {
			newNodeName = textbox.text;
		}),
		new HUD.Textbox('Package', newNodePackage, function(textbox) {
			newNodePackage = textbox.text;
		}),
		new HUD.Textbox('Executable', newNodeExecutable, function(textbox) {
			newNodeExecutable = textbox.text;
		}),
		new HUD.Textbox('Inputs (comma separated)', newNodeInputs, function(textbox) {
			newNodeInputs = textbox.text;
		}),
		new HUD.Textbox('Outputs (comma separated)', newNodeOutputs, function(textbox) {
			newNodeOutputs = textbox.text;
		})
	]);
	newNode.setButtons([
		new HUD.DefaultButton('Add', function() {
			libraryJSON.nodes.push({
				name: newNodeName,
				exec: newNodeExecutable,
				pkg: newNodePackage,
				inputs: newNodeInputs.split(/\s*,\s*/).map(function(name) {
					return { name: name };
				}),
				outputs: newNodeOutputs.split(/\s*,\s*/).map(function(name) {
					return { name: name };
				})
			});
			updateLibrary();
			newNode.hide();
		}),
		new HUD.Button('Cancel', function() {
			newNode.hide();
		})
	]);

	// populate the node library
	channel('workspace', 'library', 'response').subscribe(function(json) {
		libraryJSON = json;
		updateLibrary();
	});
	channel('workspace', 'library', 'request').publish({});

	function updateLibrary() {
		var contents = [];
		for (var i = 0; i < libraryJSON.nodes.length; i++) {
			var node = libraryJSON.nodes[i];
			var name = node.name;
			var path = node.pkg + '/' + (node.exec || node.launch);
			contents.push(new HUD.Row('<div class="name">' + name + '</div><div class="path">' + path + '</div>'));
		}
		library.setContents(contents);
	}
});
