function updateProjectList(projects) {
	var html = '';
	projects.sort();
	
	html += '<div class="mac-button newproject">Create a new project</div>';
	html += '<h2>Projects:</h2>';
	for (var i = 0; i < projects.length; i++) {
		var project = projects[i];
		html += '<div class="project"><span class="project-link">' + project.name + '</span></div>';
	}
	if (projects.length == 0) {
		html += '<div class="noprojects">No projects yet</div>';
	}
	
	$('.projects').html(html);
}

$(window).load(function() {
	var contents = [
		new Toolbar.Button('Settings', '/static/images/settings.png').floatRight()
	];

	$('.project-link').live('click', function() {
		window.open('/project/untitled/');
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

	Channel('server', 'status').subscribe(function(data) {
		Channel('workspace', 'list').publish({});
	});

	Channel('workspace', 'list').subscribe(function(data) {
		updateProjectList(data.projects);
	});
});
