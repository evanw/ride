function updateProjectList(projects) {
	var html = '';
	var names = projects.map(function(project) {
		return project.name;
	});
	names.sort();
	
	html += '<div class="mac-button newproject">Create a new project</div>';
	html += '<h2>Projects:</h2>';
	for (var i = 0; i < names.length; i++) {
		var name = names[i];
		html += '<div class="project"><span class="project-link">' + name + '</span></div>';
	}
	if (names.length == 0) {
		html += '<div class="noprojects">No projects yet</div>';
	}
	
	$('.projects').html(html);
	$('.newproject').click(function() {
		var name = window.prompt('Please enter the name of your new project (must contain only letters, numbers, and underscores):');
		if (name !== null) {
			name = $.trim(name);
			if (/^\w+$/.test(name)) {
				channel('workspace', 'list', 'add').publish({
					name: name
				});
				channel('workspace', 'list', 'request').publish({});
			} else {
				window.alert('The name "' + name + '" is invalid');
			}
		}
	});
}

$(window).load(function() {
	var contents = [
		new Toolbar.Button('Settings', '/static/images/settings.png').floatRight().setEnabled(false)
	];

	$('.project-link').live('click', function() {
		window.open('/project/' + this.innerHTML + '/');
	});
	$('.mac-button').live('mousedown', function(e) {
		e.preventDefault();
	});

	var toolbar = new Toolbar();
	toolbar.setContents(contents);

	function resize() {
		$('.contents').css({ top: toolbar.height() + 'px' });
	}
	$(window).resize(resize);
	resize();

	channel('server', 'status').subscribe(function(data) {
		channel('workspace', 'list', 'request').publish({});
	});

	channel('workspace', 'list', 'response').subscribe(function(data) {
		updateProjectList(data.projects);
	});
});
