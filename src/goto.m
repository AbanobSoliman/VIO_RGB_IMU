function goto(pointer,file)
	% GOTO Go to a Specific Line or Label in current m File
	%
	% Usage:
	%
	% goto(pointer,file)
	% return
	%
	% All goto() commands need to be followed by a 'return', always!
	%
	% The function's first input, pointer, can either be an integer value
	% representing a line in the current code to go to, or a string
	% indicating the pointer 'label' in which to go to. Alternatively,
	% pointer can be a vector array of line numbers to jump to
	% consecutively, or a cell array consisting of label strings. Setting
	% pointer to NaN jumps to a random line in the code and setting it to
	% inf jumps to the end of the code.
	%
	% The second input, file, specifies the file name of the code in which
	% the jump is headed to. If it is not specified, then it will take the
	% default value of the file name of the file that launched the goto()
	% command (i.e. the current m file).
	%
	% If the linenumber approach is used, goto() could direct to a command,
	% an if/for/while statement, inside a loop or an if, or anywhere inside
	% the current m file.
	%
	% If the pointer string approach is used, then the code finds out on
	% which line in the current code the selected pointer label string lies
	% in, and this line number is then used in the goto() algorithm. This
	% approach makes it easier to use goto(), since adding additional
	% lines of code in between goto()'s changes the line numbers throughout
	% the desired m file. To reduce confuction, the labels in the m file
	% need to be of the following syntax:
	%
	% % LABEL labelstring
	%
	% Twenty four Example files are accompanied with this file. The first
	% 15 examples use the line number input approach, while examples 16-18
	% address the pointer string approach. Examples 19-24 demonstrate
	% goto's ability to handle numeric vector or string cell inputs.
	% Check the usage of this goto() inside them in order to understand how
	% to implement goto() in your own routines.
	%
	% Example 1:
	%
	% 1- % this is a while loop using goto()
	% 2-
	% 3- a = 5;
	% 4- a = a - 1;
	% 5- disp(a)
	% 6- if a > 0
	% 7- 	goto(4)
	% 8- 	return
	% 9- end
	%
	%
	% Example 15:
	%
	% 	while true
	% 		for h = 1:1e9
	% 			for k = 1:1e9
	% 				if 1 == 1
	% 					if round(sum(double('life'))/10) == 42
	% 						goto(16)
	% 						return
	% 					end
	% 				end
	% 				pause(1)
	% 			end
	% 		end
	% 	end
	%
	% 16- msgbox('It worked!')
	%
	%
	% Example 18:
	%
	% 	 a = 0;
	%
	%    % LABEL Start
	%
	%    a = a + 1;
	%    disp(a)
	%
	%    if a < 10
	%        goto('Start')
	%        return
	%    else
	%        goto('End')
	%        return
	%    end
	%
	%    % LABEL End
	%
	%
	%
	% Example 22:
	%
	% 1-   a = 0;
	% 2-
	% 3-  goto([6,6,6,6,6,6,9])
	% 4-  return
	% 5-
	% 6-  a = a + 1;
	% 7-  goto('End')
	% 8-  return
	% 9-  msgbox(num2str(a))
	% 10-
	% 11- % LABEL End
	%
	%
	%
	% Example 23:
	%
	%     labels = {'add';'init1';'add';'add';'show'};
	%
	%     goto(labels)
	%     return
	%
	%     % LABEL init1
	%     a = 0;
	%     goto('End')
	%     return
	%
	%     % LABEL add
	%     a = a + 1;
	%     goto('End')
	%     return
	%
	%     % LABEL init2
	%     a = 0;
	%     goto('End')
	%     return
	%
	%     % LABEL show
	%     msgbox(num2str(a))
	%
	%     % LABEL End
	%
	% 
	% Example 24:
	%
	%     a = 10;
	%     goto('show','Example23')
	%     return
	%
	%
	%
	% Tested on MATLAB R2007b (7.5). Author is not responsible for any
	% problems caused by this function.
	%
	% Husam Aldahiyat, numandina@gmail.com, Jordan, 2010
	%
	
	if ~exist('pointer','var')
		error('Please provide goto() with an input')
	end
	
	if exist('file','var')
		name = file;
	else
		h = dbstack;
		name = h(end).name;
	end
	
	fid = fopen([name,'.m']);
	
	CC = textscan(fid,'%s','delimiter','\n','whitespace','');
	
	funz = strtrim(CC{1});
	
	funz = cellfun(@(x)[x,repmat(' ',1,10)],funz,'uniform',0);
	
	if ~ischar(pointer) && numel(pointer) > 1
		
		if iscell(pointer)
			
			for c = 1:length(pointer)
				evalin('caller',['goto(''',pointer{c},''')'])
			end
			
		else
			
			for c = 1:length(pointer)
				evalin('caller',['goto(',num2str(pointer(c)),')'])
			end
			
		end
		
		return
	end
	
	if ischar(pointer)
		label_lines = find(~cellfun(@isempty,...
			(cellfun(@(x)strfind(x,'% LABEL '),funz,'un',0))));
		indx = cellfun(@(x)strcmp(strtrim(x),pointer),...
			cellfun(@(x)x(9:end),funz(label_lines),'un',0));
		if sum(indx) > 1
			error('Ambiguous Label Name')
		end
		pointer = label_lines(indx);
	end
	
	if isnan(pointer)
		pointer = ceil(length(funz));
	end
	
	if isinf(pointer)
		return
	end
	
	while true
		
		if pointer > length(funz)
			break
		end
		
		if (strcmp(funz{pointer}(1:6),'return')...
				&& length(funz{pointer})==6)...
				|| strcmp(funz{pointer}(1:7),'return ')
			return
		end
		
		if strcmp(funz{pointer}(1:5),'else ')
			
			while ~((strcmp(funz{pointer}(1:3),'end')...
					&& length(funz{pointer})==3)...
					|| strcmp(funz{pointer}(1:4),'end '))
				pointer = pointer + 1;
			end
			
			pointer = pointer + 1;
			
		end
		
		if strcmp(funz{pointer}(1:3),'if ')
			cond = evalin('caller',funz{pointer}(3:end));
			if cond
				pointer = pointer + 1;
			else
				while ~cond
					pointer = pointer + 1;
					if strcmp(funz{pointer}(1:7),'elseif ')
						cond = evalin('caller',funz{pointer}(7:end));
					else
						if strcmp(funz{pointer}(1:5),'else ')
							cond = 1;
						end
					end
					if ((strcmp(funz{pointer}(1:3),'end')...
							&& length(funz{pointer})==3)...
							|| strcmp(funz{pointer}(1:4),'end '))
						break
					end
				end
				pointer = pointer + 1;
			end
			continue
			
		end
		
		if pointer > length(funz)
			break
		end
		
		if (strcmp(funz{pointer}(1:4),'for '))
			
			
			bef = pointer;
			
			extra = 0;
			while true
				pointer = pointer + 1;
				
				if (strcmp(funz{pointer}(1:3),'if '))...
						|| (strcmp(funz{pointer}(1:4),'for '))...
						|| (strcmp(funz{pointer}(1:6),'while '))
					extra = extra + 1;
				end
				
				if ((strcmp(funz{pointer}(1:3),'end')...
						&& length(funz{pointer})==3)...
						|| strcmp(funz{pointer}(1:4),'end '))
					if ~extra
						pointer = pointer + 1;
						break
					else
						extra = extra - 1;
					end
				end
			end
			
			np = bef;
			
			range = funz{np}(1:end);
			
			indz = strfind(range,'=');
			
			var = funz{np}(4:indz(1)-1);
			
			num_range = str2num(funz{np}(indz(1)+1:end)); %#ok<ST2NM>
			
			if ~isempty(num_range)
				
				pointer = np + 1;
				
				evalin('caller',[var,' = ',num2str(num_range(1)),';']);
				
			end
			
			continue
			
		end
		
		if pointer > length(funz)
			break
		end
		
		if (strcmp(funz{pointer}(1:5),'goto('))
			evalin('caller',funz{pointer})
			return
		end
		
		if (strcmp(funz{pointer}(1:6),'while '))
			
			bef = pointer;
			
			extra = 0;
			while true
				
				pointer = pointer + 1;
				
				if (strcmp(funz{pointer}(1:3),'if '))...
						|| (strcmp(funz{pointer}(1:4),'for '))...
						|| (strcmp(funz{pointer}(1:6),'while '))
					extra = extra + 1;
				end
				
				if ((strcmp(funz{pointer}(1:3),'end')...
						&& length(funz{pointer})==3)...
						|| strcmp(funz{pointer}(1:4),'end '))
					if ~extra
						pointer = pointer + 1;
						break
					else
						extra = extra - 1;
					end
				end
				
			end
			
			np = bef;
			
			cond = evalin('caller',funz{np}(6:end));
			
			if cond
				pointer = np + 1;
			end
			
			continue
			
		end
		
		if pointer > length(funz)
			break
		end
		
		if ((strcmp(funz{pointer}(1:5),'break')...
				&& length(funz{pointer})==5)...
				|| strcmp(funz{pointer}(1:6),'break '))
			
			np = pointer;
			
			extra = 0;
			while ~(strcmp(funz{np}(1:6),'while '))...
					&& ~(strcmp(funz{np}(1:4),'for '))
				
				np = np - 1;
				
				if (strcmp(funz{np}(1:3),'if '))
					extra = extra + 1;
				end
				
			end
			
			while true
				pointer = pointer + 1;
				
				if (strcmp(funz{pointer}(1:3),'if '))...
						|| (strcmp(funz{pointer}(1:4),'for '))...
						|| (strcmp(funz{pointer}(1:6),'while '))
					extra = extra + 1;
				end
				
				if ((strcmp(funz{pointer}(1:3),'end')...
						&& length(funz{pointer})==3)...
						|| strcmp(funz{pointer}(1:4),'end '))
					if ~extra
						pointer = pointer + 1;
						break
					else
						extra = extra - 1;
					end
				end
				
			end
			continue
		end
		
		if pointer > length(funz)
			break
		end
		
		if ((strcmp(funz{pointer}(1:8),'continue')...
				&& length(funz{pointer})==8)...
				|| strcmp(funz{pointer}(1:9),'continue '))
			
			extra1 = 0;
			extra2 = 0;
			np = pointer;
			
			while ~(strcmp(funz{np}(1:6),'while '))...
					&& ~(strcmp(funz{np}(1:4),'for '))
				
				np = np - 1;
				
				if ((strcmp(funz{np}(1:3),'end')...
						&& length(funz{np})==3)...
						|| strcmp(funz{np}(1:4),'end '))
					extra1 = extra1 + 1;
				end
				
				if strcmp(funz{np}(1:3),'if ') && ~extra1
					extra2 = extra2 + 1;
				end
				
				if strcmp(funz{np}(1:4),'for ')
					if ~extra1
						break
					else
						extra1 = extra1 - 1;
					end
				end
				
				if strcmp(funz{np}(1:6),'while ')
					if ~extra1
						break
					else
						extra1 = extra1 - 1;
					end
				end
				
			end
			
			while true
				pointer = pointer + 1;
				
				if (strcmp(funz{pointer}(1:3),'if '))...
						|| (strcmp(funz{pointer}(1:4),'for '))...
						|| (strcmp(funz{pointer}(1:6),'while '))
					extra2 = extra2 + 1;
				end
				
				if ((strcmp(funz{pointer}(1:3),'end')...
						&& length(funz{pointer})==3)...
						|| strcmp(funz{pointer}(1:4),'end '))
					if ~extra2
						pointer = pointer + 1;
						break
					else
						extra2 = extra2 - 1;
					end
				end
			end
			
			if (strcmp(funz{np}(1:6),'while '))
				
				cond = evalin('caller',funz{np}(6:end));
				
				if cond
					pointer = np + 1;
				end
				
			end
			
			if (strcmp(funz{np}(1:4),'for '))
				
				range = funz{np}(1:end);
				
				indz = strfind(range,'=');
				
				var = funz{np}(4:indz(1)-1);
				
				getout = 0;
				
				try
					varh = evalin('caller',var);
				catch %#ok<CTCH>
					getout = 1;
				end
				
				if ~getout
					
					num_range = str2num(funz{np}...
						(indz(1)+1:end)); %#ok<ST2NM>
					
					where_var = find(num_range == varh);
					
					if ~(where_var == length(num_range)...
							|| isempty(num_range))
						
						pointer = np + 1;
						evalin('caller',[var,' = ',...
							num2str(num_range(where_var + 1)),';']);
						
					end
					
				else
					
					num_range = str2num(funz{np}...
						(indz(1)+1:end)); %#ok<ST2NM>
					
					if ~isempty(num_range)
						
						evalin('caller',...
							[var,' = ',num2str(num_range(1)),';']);
						pointer = np + 1;
						
					end
					
				end
				
			end
			
			continue
		end
		
		if pointer > length(funz)
			break
		end
		
		if ((strcmp(funz{pointer}(1:3),'end')...
				&& length(funz{pointer})==3)...
				|| strcmp(funz{pointer}(1:4),'end '))
			np = pointer;
			permission = 1;
			
			while np > 1
				np = np - 1;
				
				if (strcmp(funz{np}(1:9),'function '))
					return
				end
				
				if (strcmp(funz{np}(1:3),'if '))
					
					if permission
						pointer = pointer + 1;
						break
					else
						permission = permission + 1;
					end
					
				end
				
				if pointer > length(funz)
					break
				end
				
				if (strcmp(funz{np}(1:4),'for '))
					
					if permission
						range = funz{np}(1:end);
						
						indz = strfind(range,'=');
						
						var = funz{np}(4:indz(1)-1);
						
						varh = evalin('caller',var);
						
						num_range = str2num(funz{np}...
							(indz(1)+1:end)); %#ok<ST2NM>
						
						where_var = find(num_range == varh);
						
						if where_var == length(num_range)...
								|| isempty(num_range)
							pointer = pointer + 1;
						else
							pointer = np + 1;
							evalin('caller',[var,' = ',...
								num2str(num_range(where_var + 1)),';']);
							
						end
						break
					else
						permission = permission + 1;
					end
					
				end
				
				if pointer > length(funz)
					break
				end
				
				if (strcmp(funz{np}(1:6),'while '))
					
					if permission
						cond = evalin('caller',funz{np}(6:end));
						
						if cond
							pointer = np + 1;
						else
							pointer = pointer + 1;
						end
						
						break
					else
						permission = permission + 1;
					end
				end
				
				if ((strcmp(funz{np}(1:3),'end')...
						&& length(funz{np})==3)...
						|| strcmp(funz{np}(1:4),'end '))
					permission = permission - 1;
				end
				
			end
			continue
		end
		
		if pointer > length(funz)
			break
		end
		
		evalin('caller',funz{pointer})
		
		pointer = pointer + 1;
		
		if pointer > length(funz)
			break
		end
		
	end
	
	% Thanks for Kamil Makiela for feedback that led to this line
	fclose('all');
	
end